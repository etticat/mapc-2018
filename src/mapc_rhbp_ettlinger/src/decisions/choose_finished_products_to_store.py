import random

import numpy as np

from common_utils import etti_logging
from common_utils.calc import CalcUtil
from provider.agent_info_provider import AgentInfoProvider
from provider.distance_provider import DistanceProvider
from provider.product_provider import ProductProvider
from so_data.patterns import DecisionPattern

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.decisions.hoarding_storage')


class ChooseFinishedProductsToStoreDecision(DecisionPattern):
    """
    Selects the finished products an agent has and is able to put away in storage
    """

    def __init__(self, agent_name, gather_mechanism, assembly_decision_mechanism):
        self.assembly_decision_mechanism = assembly_decision_mechanism
        self.gather_mechanism = gather_mechanism

        self.product_provider = ProductProvider(agent_name=agent_name)

        super(ChooseFinishedProductsToStoreDecision, self).__init__(buffer=None, frame=None, requres_pos=False)

    def calc_value(self):
        """
        Takes all the items in stock minus the ones that are needed for further assembly and those that are desired by
        the planner
        :return:
        """
        # Get all the items that all agents currently have in stock
        finished_product_stock = self.product_provider.get_agent_stock_items([ProductProvider.STOCK_ITEM_TYPE_STOCK, ProductProvider.STOCK_ITEM_TYPE_HOARDING_GOAL])

        # Subtract all items that we need for assembly
        desired_ingredients = self.gather_mechanism.get_desired_ingredients(consider_intermediate_ingredients=True)
        finished_products_to_store = CalcUtil.dict_diff(finished_product_stock, desired_ingredients,
                                                        normalize_to_zero=True)

        # Subtract all items that we need for delivery
        desired_finished_products = self.assembly_decision_mechanism.finished_product_goals
        finished_products_to_store = CalcUtil.dict_diff(finished_products_to_store, desired_finished_products,
                                                        normalize_to_zero=True)

        # of all the items that require storing, pick the ones that the current agent has
        finished_product_agent_stock = self.product_provider.get_finished_products_in_stock()
        finished_product_agent_stock = CalcUtil.dict_intersect(finished_products_to_store, finished_product_agent_stock)

        if sum(finished_product_agent_stock.values()) == 0:
            finished_product_agent_stock = None

        return [finished_product_agent_stock, self.state]
