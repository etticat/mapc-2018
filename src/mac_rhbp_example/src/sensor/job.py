#!/usr/bin/env python2


from __future__ import division  # force floating point division when using plain /

from abc import abstractmethod

import rospy
from mac_ros_bridge.msg import Position, Agent

from agent_common.agent_utils import AgentUtils
from agent_knowledge.movement import MovementKnowledge
from agent_knowledge.tasks import TaskKnowledge
from behaviour_components.activators import BooleanActivator
from behaviour_components.sensors import Sensor
from rhbp_utils.knowledge_sensors import KnowledgeFirstFactSensor, KnowledgeFactSensor
from knowledge_base.update_handler import KnowledgeBaseFactCache


class ProductSensor(Sensor):

    def __init__(self, agent_name, **kwargs):

        self._task_knowledge =  TaskKnowledge(agent_name=agent_name)

        self._agent_name = agent_name

        super(ProductSensor, self).__init__(**kwargs)

        self._latest_ref_value = None


    def sync(self):
        still_needed_products = self.get_still_needed_products()
        self.update(still_needed_products)

        super(ProductSensor, self).sync()

    @abstractmethod
    def get_still_needed_products(self):
        """
        :return: list
        """
        pass

class FinishedProductSensor(ProductSensor):
    def get_still_needed_products(self):
        return self._task_knowledge.get_required_finished_products(self._agent_name)

class IngredientSensor(FinishedProductSensor):

    def get_still_needed_products(self):
        return self._task_knowledge.get_required_ingredients(self._agent_name)


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
