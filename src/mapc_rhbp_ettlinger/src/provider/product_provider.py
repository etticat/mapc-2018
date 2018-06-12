#!/usr/bin/env python2
import copy
import itertools
import operator

import rospy
from mac_ros_bridge.msg import SimStart, Agent, Item
from mapc_rhbp_ettlinger.msg import StockItem, StockItemMsg

from agent_knowledge.item import StockItemKnowledgebase
from agent_knowledge.tasks import TaskKnowledgebase
from common_utils.agent_utils import AgentUtils
from common_utils.calc import CalcUtil
from common_utils.singleton import Singleton


class ProductProvider(object):
    __metaclass__ = Singleton

    def __init__(self, agent_name):

        self.products = {}
        self.base_ingredients = {}
        self.finished_products = {}
        self._items_in_stock = {}
        self._gather_goal = {}
        self._assemble_goal = {}

        self._agent_name = agent_name

        self.load_max = 0
        self.load_free = 0
        self.load = 0

        rospy.Subscriber(
            AgentUtils.get_bridge_topic_prefix(agent_name=agent_name) + "start",
            SimStart,
            self._callback_sim_start)

        self._task_knowledgebase = TaskKnowledgebase()
        self._stock_item_knowledgebase = StockItemKnowledgebase()

        self._sub_ref = rospy.Subscriber(AgentUtils.get_bridge_topic_agent(agent_name), Agent, self._callback_agent)

    def _callback_agent(self, msg):
        """
        Retreives the current state of stock items for use in other methods
        :param msg:
        :type msg: Agent
        :return:
        """
        items_in_stock_new  = {}

        for item in self.products.keys():
            items_in_stock_new[item] = 0

        for item in msg.items:
            items_in_stock_new[item.name] = items_in_stock_new.get(item.name, 0) + item.amount


        if items_in_stock_new != self._items_in_stock:
            self._items_in_stock = items_in_stock_new
            self.save_stock_and_sock_goals()

        self.load_max = msg.load_max
        self.load = msg.load
        self.load_free = msg.load_max - msg.load

    def _callback_sim_start(self, msg):
        """
        Retreives the products of the current simulation and their details
        :param msg:
        :type msg: SimStart
        :return:
        """
        for product in msg.products:
            self.products[product.name] = product
            if len(product.consumed_items) > 0:
                self.finished_products[product.name] = product
            else:
                self.base_ingredients[product.name] = product

    def get_product_by_name(self, product):
        """
        Returns the product by name
        :param product:
        :return: Product
        """
        return self.products[product]

    def get_base_ingredients(self):
        return self.base_ingredients

    def save_stock_and_sock_goals(self):
        msg = StockItemMsg()
        for index in self._items_in_stock.keys():
            item = StockItem(agent=self._agent_name, item=index, amount=self._items_in_stock[index],
                             goal=self._gather_goal.get(index, 0) + self._assemble_goal.get(index, 0))
            self._stock_item_knowledgebase.update_stock(item)
            msg.stock_items.append(item)

    def start_gathering(self, item_to_focus):

        load_free = self.load_max - self.load

        items_to_carry = load_free / self.get_product_by_name(item_to_focus).volume

        self._gather_goal = {
            item_to_focus: items_to_carry + self._items_in_stock.get(item_to_focus, 0)
        }
        self.save_stock_and_sock_goals()

    def start_assembly(self, items_to_assemble):
        self._assemble_goal = items_to_assemble
        self.save_stock_and_sock_goals()

    def stop_gathering(self):
        self._gather_goal = {}
        self.save_stock_and_sock_goals()

    def stop_assembly(self):
        self._assemble_goal = {}
        self.save_stock_and_sock_goals()

    def get_required_ingredients_for_hoarding(self):
        """
        Returns all ingredients that an agent needs to gather to meet its goal
        :return: list
        """
        res = []

        for index in self.base_ingredients:
            needed_amount = self._gather_goal.get(index, 0) - self._items_in_stock.get(index, 0)

            if needed_amount > 0:
               res.append(index)

        rospy.loginfo("All ingredients to gather: %s", str(res) )
        return res

    def get_required_finished_products_for_hoarding(self):
        """
        Returns all finished products, that an agent needs to assemble to meet its goal
        :return: list
        """
        res = []

        for index in self.finished_products:
            needed_amount = self._assemble_goal.get(index, 0) - self._items_in_stock.get(index, 0)

            if needed_amount > 0:
                res.append(index)

        return res

    def calculate_total_volume(self, product_dict):
        """
        Calculates the total volume that a dict of products will use
        :param product_dict:
        :return:
        """

        v = 0

        for product in product_dict.keys():
            product_by_name = self.get_product_by_name(product)
            v += product_by_name.volume * product_dict[product]

        return v

    def calculate_desired_finished_product_stock(self):
        # TODO: Take into account how much each each product is worth (how much it usually pays)
        # TODO: Take into account how much effort it is to make (moving, gathering, assembling)...

        res = {}

        for product in self.finished_products.keys():
            res[product] = 3

        return res

    def total_items_in_stock(self):
        return self._stock_item_knowledgebase.get_total_stock()

    def get_ingredients_of_product(self, product_name, amount = 1):
        product = self.products[product_name]

        if len(product.consumed_items) == 0:
            return {product_name:amount}
        else:
            res = {}
            for ingredient in product.consumed_items:
                res = CalcUtil.dict_sum(res, self.get_ingredients_of_product(ingredient.name, ingredient.amount * amount))
            return res

    def get_roles_of_product(self, product_name):
        product = self.products[product_name]
        return product.required_roles

    def calculate_desired_ingredient_stock(self):
        desired_finished_product_stock = self.calculate_desired_finished_product_stock()
        all_items = self.total_items_in_stock()

        ingredients = {}

        keys = desired_finished_product_stock.keys()
        for finished_product in keys:
            items_taken_from_inventory = min(desired_finished_product_stock[finished_product], all_items[finished_product])
            all_items[finished_product] -= items_taken_from_inventory
            desired_finished_product_stock[finished_product] -= items_taken_from_inventory


            ingredients = CalcUtil.dict_sum(self.get_ingredients_of_product(finished_product, desired_finished_product_stock[finished_product]), ingredients)

        # TODO: Take into account products that are assembled from already assembled products

        keys = ingredients.keys()
        for ingredient in keys:
            items_taken_from_inventory = min(ingredients[ingredient], all_items[ingredient])
            ingredients[ingredient] -= items_taken_from_inventory

        return ingredients

    def get_items(self):
        res = []
        for item, amount in self._items_in_stock.iteritems():
            if amount > 0:
                res.append(Item(
                    name=item,
                    amount=amount
                ))

        return res
    

    def choose_best_bid_combination(self, bids):

        best_combination = []
        best_value = 0
        best_finished_products = {}

        if len(bids) >= 2:
            # Go through all combinations
            for L in range(2, len(bids) + 1):
                for subset in itertools.combinations(bids, L):
                    stringi = ""
                    for item in subset:
                        stringi = stringi + item.agent_name + "(" + item.role + ")" + " - "

                    combination = self.generate_best_combination(subset)

                    value = self.generate_value_from_combination(combination)

                    # rospy.logerr(stringi + str(value) + str(combination))

                    if value > best_value:
                        best_value = value
                        best_combination = subset
                        best_finished_products = combination

        return (best_combination, best_finished_products)

    def generate_best_combination(self, subset):
        item_dict = {}
        roles = []

        for bid in subset:
            for item in bid.items:
                item_dict[item.name] = item_dict.get(item.name, 0) + item.amount
            roles.append(bid.role)

        finished_products = self.generate_best_finished_product_combination(item_dict, roles)

        return finished_products
    


    def get_product_worth(self, product):
        # TODO: Temporarily
        # This will be replaced with something that learns
        product_worth = {
            "item0": 21.1214,
            "item1": 17.1214,
            "item3": 28.1214,
            "item4": 19.1214,
        }
        return product_worth[product]


    def get_goal_stock(self):
        # TODO: Temporarily
        # This will be replaced with something that learns

        goal_stock = {
            "item5": 9,
            "item6": 4,
            "item7": 3,
            "item8": 8,
            "item9": 2,
            "item10": 5,
        }

        return goal_stock

    def update_product_worth(self, job):
        # TODO: Use jobs to check item worth
        # TODO: Also update ideal number of items
        pass

    def generate_best_finished_product_combination(self, item_dict, roles):

        item_dict = copy.copy(item_dict)
        combination = {}

        for item, count in reversed(sorted(self.get_goal_stock().items(), key=operator.itemgetter(1))):
            required_roles = self.get_roles_of_product(item)
            if set(required_roles).issubset(roles):
                ingredients = self.get_ingredients_of_product(item)
                item_count = CalcUtil.dict_max_diff(ingredients, item_dict)

                if item_count > 0:
                    combination[item] = item_count

                item_dict = CalcUtil.dict_diff(item_dict, self.get_ingredients_of_product(item, item_count))

        return combination

    def generate_value_from_combination(self, combination):
        # TODO: There needs to be a better logic behind this

        value = 0

        for item, count in combination.iteritems():
            value += self.get_goal_stock().get(item, 0) * count

        return value

