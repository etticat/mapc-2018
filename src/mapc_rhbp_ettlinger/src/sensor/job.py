#!/usr/bin/env python2


from __future__ import division  # force floating point division when using plain /

from abc import abstractmethod

import rospy

from behaviour_components.activators import BooleanActivator
from behaviour_components.sensors import Sensor
from provider.product_provider import ProductProvider


class ProductSensor(Sensor):

    def __init__(self, agent_name, **kwargs):

        self._product_provider = ProductProvider(agent_name=agent_name)

        self._agent_name = agent_name

        super(ProductSensor, self).__init__(**kwargs)

        self._latest_ref_value = None


    def sync(self):
        still_needed_products = self.get_still_needed_products()
        self.update(still_needed_products)

        super(ProductSensor, self).sync()

    def get_still_needed_products(self):
        products = self._product_provider.get_planned_ingredients()
        rospy.loginfo("%s:: Need following products:  %s",self._name, str(products))
        return products


class AmountInListActivator(BooleanActivator):

    def __init__(self, amount, desired_value=True, min_activation=0, max_activation=1, name=None):

        super(AmountInListActivator, self).__init__(
            desiredValue = desired_value,
            minActivation = min_activation,
            maxActivation = max_activation,
            name = name)
        self.amount = amount

    def computeActivation(self, value_list):
        if self.has_enough(value_list):
            return self._maxActivation
        else:
            return self._minActivation

    def getDirection(self):
        return 1.0

    def getSensorWish(self, value_list):
        has_enough = self.has_enough(value_list)
        if has_enough:
            return 0.0
        else:
            return 1.0

    def has_enough(self, value_list):
        amount = len(value_list) <= self.amount
        return amount

