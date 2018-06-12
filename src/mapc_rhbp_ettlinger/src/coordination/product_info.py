import copy
import itertools
import operator
import time

import rospy
from mapc_rhbp_ettlinger.msg import AssembleRequest, AssembleBid, AssembleAssignment, AssembleAcknowledgement

from common_utils.agent_utils import AgentUtils
from common_utils.calc import CalcUtil
from provider.product_provider import ProductProvider


class ProductValueInfo(object):

    def __init__(self):

        self._product_provider = ProductProvider(agent_name="agentA1")


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

    def choose_best_bid_combination(self, bids, manager_items, manager_role):

        best_combination = []
        best_value = 0
        best_finished_products = {}

        # Go through all combinations
        for L in range(1, len(bids) + 1):
            for subset in itertools.combinations(bids, L):
                stringi = "agentA1(" + manager_role + ") - "
                for item in subset:
                    stringi = stringi + item.agent_name + "(" + item.role + ")" + " - "

                combination = self.generate_best_combination(subset, manager_items, manager_role)

                value = self.generate_value_from_combination(combination)

                # if value > 0:
                #     rospy.logerr(stringi + str(value))

                if value > best_value:
                    best_value = value
                    best_combination = subset
                    best_finished_products = combination


        # combine all bids
        # for i in xrange(len(bids)):
        #     for j in xrange(len(bids)):
        #         for k in xrange(len(bids)):
        #             for l in xrange(len(bids)):
        #                 current_combination = [i,j,k,l]
        #                 current_bid = self.calculate_assist_team_value([i,j,k,l])
        #
        #                 if current_bid > max_bid:
        #                     combination = current_combination
        #                     max_bid = current_bid
        # self.assign_bids(bids, combination)
        # TODO: Calculate best possible combination

        return (best_combination, best_finished_products)

    def generate_best_combination(self, subset, manager_items, manager_role):
        item_dict = {}
        roles = []

        for item in manager_items:
            item_dict[item.name] = item_dict.get(item.name, 0) + item.amount

        roles.append(manager_role)

        for bid in subset:
            for item in bid.items:
                item_dict[item.name] = item_dict.get(item.name, 0) + item.amount
            roles.append(bid.role)

        finished_products = self.generate_best_finished_product_combination(item_dict, roles)

        # for finished_product in self._product_provider.finished_products.keys():
        #     ingredients = self._product_provider.get_ingredients_of_product(finished_product)


        return finished_products

    def generate_best_finished_product_combination(self, item_dict, roles):

        item_dict = copy.copy(item_dict)
        combination = {}

        for item, count in reversed(sorted(self.get_goal_stock().items(), key=operator.itemgetter(1))):
            required_roles = self._product_provider.get_roles_of_product(item)
            ingredients = self._product_provider.get_ingredients_of_product(item)

            all_roles_available = True
            for required_role in required_roles: # TODO: Do this more elegant
                if required_role not in roles:
                    all_roles_available = False

            if all_roles_available == False:
                continue
            item_count = CalcUtil.dict_max_diff(ingredients, item_dict)

            if item_count > 0:
                combination[item] = item_count

            item_dict = CalcUtil.dict_diff(item_dict, self._product_provider.get_ingredients_of_product(item, item_count))

        return combination

    def generate_value_from_combination(self, combination):
        # TODO: There needs to be a better logic behind this

        value = 0

        for item, count in combination.iteritems():
            value += self.get_goal_stock().get(item, 0) * count

        return value