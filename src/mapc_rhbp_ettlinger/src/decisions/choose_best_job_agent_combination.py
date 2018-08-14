import copy
import itertools

import rospy
from mac_ros_bridge.msg import SimStart

from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from common_utils.calc import CalcUtil
from provider.facility_provider import FacilityProvider
from provider.product_provider import ProductProvider
from provider.simulation_provider import SimulationProvider
from provider.stats_provider import StatsProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.decisions.job_combination')


class ChooseBestJobAgentCombinationDecision(object):
    """
    Chooses the best combination of agents to start a delivery job
    """
    MIN_STEP_BUFFER = 5

    WEIGHT_LOAD = 8
    WEIGHT_INGREDIENT_LOAD = 8
    WEIGHT_NO_STORAGE_NEEDED = 20
    WEIGHT_STEPS = -1

    ACTIVATION_THRESHOLD = -1000

    def __init__(self, agent_name):

        self._init_config()

        self._product_provider = ProductProvider(agent_name=agent_name)
        self._facility_provider = FacilityProvider(agent_name=agent_name)
        self._simulation_provider = SimulationProvider(agent_name=agent_name)
        # Reload config after each simulation start
        self._sub_ref = rospy.Subscriber(AgentUtils.get_bridge_topic(agent_name, postfix="start"), SimStart, self._init_config)

    def _init_config(self, sim_start=None):

        ChooseBestJobAgentCombinationDecision.MIN_STEP_BUFFER = rospy.get_param(
            "ChooseBestJobAgentCombinationDecision.MIN_STEP_BUFFER",
            ChooseBestJobAgentCombinationDecision.MIN_STEP_BUFFER)
        ChooseBestJobAgentCombinationDecision.WEIGHT_LOAD = rospy.get_param(
            "ChooseBestJobAgentCombinationDecision.WEIGHT_LOAD", ChooseBestJobAgentCombinationDecision.WEIGHT_LOAD)
        ChooseBestJobAgentCombinationDecision.WEIGHT_INGREDIENT_LOAD = rospy.get_param(
            "ChooseBestJobAgentCombinationDecision.WEIGHT_INGREDIENT_LOAD",
            ChooseBestJobAgentCombinationDecision.WEIGHT_INGREDIENT_LOAD)
        ChooseBestJobAgentCombinationDecision.WEIGHT_NO_STORAGE_NEEDED = rospy.get_param(
            "ChooseBestJobAgentCombinationDecision.WEIGHT_NO_STORAGE_NEEDED",
            ChooseBestJobAgentCombinationDecision.WEIGHT_NO_STORAGE_NEEDED)
        ChooseBestJobAgentCombinationDecision.WEIGHT_STEPS = rospy.get_param(
            "ChooseBestJobAgentCombinationDecision.WEIGHT_STEPS", ChooseBestJobAgentCombinationDecision.WEIGHT_STEPS)
        ChooseBestJobAgentCombinationDecision.ACTIVATION_THRESHOLD = rospy.get_param(
            "ChooseBestJobAgentCombinationDecision.ACTIVATION_THRESHOLD",
            ChooseBestJobAgentCombinationDecision.ACTIVATION_THRESHOLD)

    def choose_best_agent_combination(self, job, bids):
        """
        Chooses the best agent combination to fulfill a job from their bids
        :param job:
        :param bids:
        :return:
        """
        best_agent_subset = None
        items_needed_from_storage = None
        best_value = -1000

        items_in_storage = self._product_provider.get_stored_items(storage_name=job.storage_name,
                                                                   include_hoarding_goal=False)
        job_items = CalcUtil.get_list_from_items(job.items)

        if len(bids) >= 1:
            # Go through all combinations
            for L in range(1, min(len(bids) + 1, 7)):  # We try all combinations using 1-7 agents
                for subset in itertools.combinations(bids, L):

                    # Check which items are still required after all agents would deliver the items
                    still_required_items = self.get_still_needed_items_after_bid_execution(job_items, subset)
                    can_fulfill_job_without_storage_items = sum(still_required_items.values()) == 0

                    # Check which items are needed after also providing storage items
                    items_needed_after_storage_emptying = CalcUtil.dict_diff(still_required_items, items_in_storage,
                                                                             normalize_to_zero=True)
                    can_fulfill_job = sum(items_needed_after_storage_emptying.values()) == 0

                    if can_fulfill_job:
                        value = self.generate_activation(subset, job, can_fulfill_job_without_storage_items)

                        if value > best_value:
                            best_value = value

                        best_agent_subset = subset
                        items_needed_from_storage = still_required_items

                if best_value > ChooseBestJobAgentCombinationDecision.ACTIVATION_THRESHOLD:
                    # we only try combinations with more, if we could not find anything with less
                    break

        return best_agent_subset, items_needed_from_storage

    def get_still_needed_items_after_bid_execution(self, job_items, bids):
        """
        Returns the items that are still needed after all bids are delivered
        :param job_items:
        :param bids:
        :return:
        """
        job_items = copy.copy(job_items)

        for bid in bids:

            useful_items = CalcUtil.list_intersect(job_items, bid.items)
            if len(useful_items) > 0:
                job_items = CalcUtil.list_diff(job_items, useful_items)

        return CalcUtil.dict_from_string_list(job_items)

    def generate_activation(self, subset, job, can_fulfill_job_without_storage_items):
        """
        Generates the activation for a job
        :param subset:
        :param job:
        :param can_fulfill_job_without_storage_items:
        :return:
        """

        # The number of step until all agents can be at the workshop
        max_step_count = max([bid.expected_steps for bid in subset])

        if job.end - self._simulation_provider.step < max_step_count + ChooseBestJobAgentCombinationDecision.MIN_STEP_BUFFER:
            return 0  # This combination of agents can't make it in time

        activation = max_step_count * ChooseBestJobAgentCombinationDecision.WEIGHT_STEPS

        if can_fulfill_job_without_storage_items:
            activation += ChooseBestJobAgentCombinationDecision.WEIGHT_NO_STORAGE_NEEDED

        return activation

