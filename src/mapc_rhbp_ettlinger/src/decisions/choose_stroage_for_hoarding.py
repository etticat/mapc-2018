import numpy as np

import rospy
from mac_ros_bridge.msg import Agent, SimStart

from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from provider.agent_info_provider import AgentInfoProvider
from provider.distance_provider import DistanceProvider
from provider.facility_provider import FacilityProvider
from provider.product_provider import ProductProvider
from so_data.patterns import DecisionPattern

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.decisions.hoarding_storage')


class ChooseStorageForHoardingDecision(DecisionPattern):
    """
    Decides for a storage to store excess finished products into
    """

    WEIGHT_STEPS = -1
    WEIGHT_ITEMS_ALREADY_THERE = -1

    def __init__(self, agent_name, assembly_decision_mechanism, hoarding_items_decision):

        self._agent_name = agent_name
        self._hoarding_items_decision = hoarding_items_decision
        self._assembly_decision_mechanism = assembly_decision_mechanism

        self._init_config()

        self._distance_provider = DistanceProvider(agent_name=agent_name)
        self._agent_info_provider = AgentInfoProvider(agent_name=agent_name)
        self._product_provider = ProductProvider(agent_name=agent_name)
        self._facility_provider = FacilityProvider(agent_name=agent_name)

        super(ChooseStorageForHoardingDecision, self).__init__(buffer=None, frame=None, requres_pos=False)

        # Reload config after each simulation start
        self._sub_ref = rospy.Subscriber(AgentUtils.get_bridge_topic(agent_name, postfix="start"), SimStart, self._init_config)

    def _init_config(self, sim_start=None):

        ChooseStorageForHoardingDecision.WEIGHT_STEPS = rospy.get_param(
            "ChooseStorageForHoardingDecision.WEIGHT_STEPS", ChooseStorageForHoardingDecision.WEIGHT_STEPS)
        ChooseStorageForHoardingDecision.WEIGHT_ITEMS_ALREADY_THERE = rospy.get_param(
            "ChooseStorageForHoardingDecision.WEIGHT_ITEMS_ALREADY_THERE",
            ChooseStorageForHoardingDecision.WEIGHT_ITEMS_ALREADY_THERE)

    def calc_value(self):
        """
        Gets the items that need to be stored and picks a storage that can make most use of those items
        :return:
        """
        max_activation = -np.inf
        chosen_storage = None

        # Get the products to store
        finished_products_to_store = self._hoarding_items_decision.calc_value()[0]
        assert finished_products_to_store is not None

        # Try all agents and pick the one with the highest activation
        for storage_name, storage in self._facility_provider.get_storages().iteritems():

            items_in_storage = self._product_provider.get_stored_items(storage_name=storage_name)
            activation = 0

            for product, count in finished_products_to_store.iteritems():
                activation += count * items_in_storage.get(product,
                                                           0) * ChooseStorageForHoardingDecision.WEIGHT_ITEMS_ALREADY_THERE

            activation += self._distance_provider.calculate_steps(self._agent_info_provider.pos,
                                                                  storage.pos) * ChooseStorageForHoardingDecision.WEIGHT_STEPS

            if activation > max_activation:
                max_activation = activation
                chosen_storage = storage

        self.value = chosen_storage

        ettilog.loginfo("ChooseStorageForHoardingDecision(%s):: Choosing storage %s ", self._agent_name,
                       self.value.name)

        # Update hoarding goal
        self._product_provider.update_hoarding_goal(finished_products_to_store, destination=self.value.name)

        return [self.value, self.state]

    def reset_value(self):
        """
        Reset hoarding mechanism when hoarding is over or interrupted
        :return:
        """

        if self.value is not None:
            self._product_provider.stop_hoarding(destination=self.value.name)
        self.value = None
