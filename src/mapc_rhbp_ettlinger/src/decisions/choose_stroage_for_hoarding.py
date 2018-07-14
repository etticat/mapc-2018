import random

import numpy as np

from common_utils import etti_logging
from common_utils.calc import CalcUtil
from provider.agent_info_provider import AgentInfoProvider
from provider.distance_provider import DistanceProvider
from provider.facility_provider import FacilityProvider
from provider.product_provider import ProductProvider
from so_data.patterns import DecisionPattern

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.decisions.hoarding_storage')

class ChooseStorageForHoardingMechanism(DecisionPattern):

    WEIGHT_STEPS = -1
    WEIGHT_ITEMS_ALREADY_THERE = -1

    def __init__(self, agent_name, assembly_decision_mechanism, hoarding_items_decision):
        self.hoarding_items_decision = hoarding_items_decision
        self.assembly_decision_mechanism = assembly_decision_mechanism
        self._agent_name = agent_name
        self.distance_provider = DistanceProvider()
        self.agent_info_provider = AgentInfoProvider(agent_name=agent_name)
        self.product_provider = ProductProvider(agent_name=agent_name)
        self.facility_provider = FacilityProvider()

        super(ChooseStorageForHoardingMechanism, self).__init__(buffer=None, frame=None, requres_pos=False)

    def calc_value(self):
        max_activation = -np.inf
        chosen_storage = None
        finished_products_to_store = self.hoarding_items_decision.calc_value()[0]

        assert finished_products_to_store is not None

        for storage_name, storage in self.facility_provider.get_storages().iteritems():
            items_in_storage = self.product_provider.get_stored_items(storage_name=storage_name)
            activation = 0
            for product, count in finished_products_to_store.iteritems():
                activation += count * items_in_storage.get(product, 0) * ChooseStorageForHoardingMechanism.WEIGHT_ITEMS_ALREADY_THERE
            activation += self.distance_provider.calculate_steps(self.agent_info_provider.pos, storage.pos) * ChooseStorageForHoardingMechanism.WEIGHT_STEPS

            if activation > max_activation:
                max_activation = activation
                chosen_storage = storage

        self.value = chosen_storage
        ettilog.logerr("ChooseStorageForHoardingMechanism(%s):: Choosing storage %s ", self._agent_name, self.value.name)

        self.product_provider.update_hoarding_goal(finished_products_to_store, destination=self.value.name)

        return [self.value, self.state]

    def reset_value(self):
        if self.value is not None:
            self.product_provider.stop_hoarding(destination=self.value.name)
        self.value = None