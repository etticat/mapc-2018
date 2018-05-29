#!/usr/bin/env python2


from __future__ import division  # force floating point division when using plain /

import rospy
from mac_ros_bridge.msg import Position, Agent

from agent_common.agent_utils import AgentUtils
from agent_knowledge.movement import MovementKnowledge
from agent_knowledge.tasks import TaskKnowledge
from behaviour_components.activators import BooleanActivator
from behaviour_components.sensors import Sensor
from rhbp_utils.knowledge_sensors import KnowledgeFirstFactSensor, KnowledgeFactSensor
from knowledge_base.update_handler import KnowledgeBaseFactCache


class FinishedProductSensor(Sensor):

    def __init__(self, agent_name, **kwargs):

        pattern = TaskKnowledge.get_tuple_task_creation(agent=agent_name, status="assigned")

        super(FinishedProductSensor, self).__init__(**kwargs)

        self._latest_ref_value = None
        self._items_in_stock = {}
        self._assembled_items = {}

        self._sub_ref = rospy.Subscriber(AgentUtils.get_bridge_topic_agent(agent_name), Agent, self.subscription_callback_ref_topic)

        self._value_cache = KnowledgeBaseFactCache(pattern=pattern, knowledge_base_name="knowledgeBaseNode")

    def subscription_callback_ref_topic(self, msg):
        """

        :param msg:
        :type msg: Agent
        :return:
        """
        self._items_in_stock  = {}

        for item in msg.items:
            self._items_in_stock[item.name] = self._items_in_stock.get(item.name, 0) + item.amount

    def sync(self):
        (still_needed_products, stock_items) = self.get_still_needed_items()
        self.update(still_needed_products)

        if(len(still_needed_products) > 0):
            rospy.logerr("%s::%s", self.name, still_needed_products)
        super(FinishedProductSensor, self).sync()

    def get_still_needed_items(self):
        required_finished_products = self._value_cache.get_all_matching_facts()
        items_in_stock = self._items_in_stock
        still_needed_products = {}
        for item in required_finished_products:
            item_name = item[6]

            if (items_in_stock.get(item_name, 0) > 0):
                # we already have the item
                items_in_stock[item_name] -= 1
                continue
            else:
                # we need the item
                still_needed_products[item_name] = still_needed_products.get(item_name, 0) + 1
        return still_needed_products, items_in_stock

class IngredientSensor(FinishedProductSensor):

    def __init__(self, agent_name, **kwargs):

        super(IngredientSensor, self).__init__(agent_name, **kwargs)
        self.task_knowledge = TaskKnowledge(agent_name)

    def get_still_needed_items(self):
        (still_needed_finished_products, items_in_stock) = super(IngredientSensor, self).get_still_needed_items()

        products_needs = self.task_knowledge.products

        still_needed_ingredients = {}
        for item_name, count in still_needed_finished_products.items():
            item = products_needs[item_name]
            for required_ingredient in item.consumed_items:

                stock = items_in_stock.get(item_name, 0)

                if(required_ingredient.amount <= stock):
                    items_in_stock[item_name] -= required_ingredient.amount
                else:
                    still_needed_items = required_ingredient.amount - items_in_stock.get(item_name, 0)
                    items_in_stock[item_name] = 0

                    still_needed_ingredients[required_ingredient.name] = still_needed_ingredients.get(item_name, 0)+ still_needed_items

        return (still_needed_ingredients, items_in_stock)


class AmountInListActivator(BooleanActivator):

    def __init__(self, amount, desired_value=True, min_activation=0, max_activation=1, name=None):

        super(AmountInListActivator, self).__init__(
            desiredValue = desired_value,
            minActivation = min_activation,
            maxActivation = max_activation,
            name = name)
        self.amount = amount

    def computeActivation(self, value_list):
        if len(value_list) <= self.amount:
            return self._maxActivation
        else:
            return self._minActivation

    def getDirection(self):
        return 1.0

    def getSensorWish(self, value_list):
        if len(value_list) <= self.amount:
            return 0.0
        else:
            return 1.0

