import random

import numpy as np

from common_utils import etti_logging
from common_utils.calc import CalcUtil
from provider.agent_info_provider import AgentInfoProvider
from provider.distance_provider import DistanceProvider
from provider.product_provider import ProductProvider
from so_data.patterns import DecisionPattern

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.decisions.hoarding_storage')

class ChooseFinishedProductsToStore(DecisionPattern):


    def __init__(self, agent_name, gather_mechanism, assembly_decision_mechanism):
        self.assembly_decision_mechanism = assembly_decision_mechanism
        self.gather_mechanism = gather_mechanism
        self.product_provider = ProductProvider(agent_name=agent_name)

        super(ChooseFinishedProductsToStore, self).__init__(buffer=None, frame=None, requres_pos=False)

    def calc_value(self):
        # Check all the products we have in stock
        finished_product_stock = self.product_provider.get_finished_products_in_stock()
        # Check all the products we have, that can be used to make another item
        desired_ingredients = self.gather_mechanism.get_desired_ingredients(consider_intermediate_ingredients=True)
        desired_finished_products = self.assembly_decision_mechanism.get_desired_finished_products()

        finished_products_to_store = CalcUtil.dict_diff(finished_product_stock, desired_ingredients, normalize_to_zero=True)
        finished_products_to_store = CalcUtil.dict_diff(finished_products_to_store, desired_finished_products, normalize_to_zero=True)

        if sum(finished_products_to_store.values()) == 0:
            finished_products_to_store = None

        return [finished_products_to_store, self.state]
