import copy
import itertools

from common_utils import etti_logging
from common_utils.calc import CalcUtil
from provider.facility_provider import FacilityProvider
from provider.product_provider import ProductProvider
from provider.stats_provider import StatsProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.decisions.job_combination')

class ChooseBestJobCombination(object):

    MIN_STEP_BUFFER = 5

    WEIGHT_LOAD = 8
    WEIGHT_INGREDIENT_LOAD = 8
    WEIGHT_NO_STORAGE_NEEDED = 20
    WEIGHT_STEPS = -1

    ACTIVATION_THRESHOLD = -1000

    def __init__(self):

        self.product_provider = ProductProvider(agent_name="agentA1")
        self.facility_provider = FacilityProvider()
        self._stats_provider = StatsProvider()

    def choose_best_agent_combination(self, job, bids):

        items_in_storage = self.product_provider.get_stored_items(storage_name=job.storage_name)
        job_items = CalcUtil.get_list_from_items(job.items)

        best_agent_subset = None
        items_needed_from_storage = None
        best_value = -1000

        if len(bids) >= 1:
            # Go through all combinations
            for L in range(1, min(len(bids) + 1, 7)):  # We try all combinations using 1-7 agents
                for subset in itertools.combinations(bids, L):

                    still_required_items = self.job_fulfillment_possible(job_items, subset)

                    can_fulfill_job_without_storage_items = sum(still_required_items.values()) == 0

                    items_needed_after_storage_emptying = CalcUtil.dict_diff(still_required_items, items_in_storage, normalize_to_zero=True)

                    can_fulfill_job = sum(items_needed_after_storage_emptying.values()) == 0

                    if can_fulfill_job:
                        best_value = self.generate_activation(subset, job, can_fulfill_job_without_storage_items)
                        best_agent_subset = subset
                        items_needed_from_storage = still_required_items
                        ettilog.logerr("ChooseBestJobCombination:: still_required_items: %s", str(still_required_items))
                        ettilog.logerr("ChooseBestJobCombination:: items_in_storage: %s", str(items_in_storage))
                        ettilog.logerr("ChooseBestJobCombination:: can_fulfill_job_without_storage_items: %s", str(can_fulfill_job_without_storage_items))
                        ettilog.logerr("ChooseBestJobCombination:: items_needed_after_storage_emptying: %s", str(items_needed_after_storage_emptying))
                        ettilog.logerr("ChooseBestJobCombination:: can_fulfill_job: %s", str(can_fulfill_job))

                if best_value > ChooseBestJobCombination.ACTIVATION_THRESHOLD:
                    # we only try combinations with more, if we could not find anything with less
                    break

        return best_agent_subset, items_needed_from_storage


    def job_fulfillment_possible(self, job_items, bids):
        job_items =copy.copy(job_items)

        for bid in bids:

            useful_items = CalcUtil.list_intersect(job_items, bid.items)
            if len(useful_items) > 0:

                job_items = CalcUtil.list_diff(job_items, useful_items)

        return CalcUtil.dict_from_strings(job_items)

    def generate_activation(self, subset, job, can_fulfill_job_without_storage_items):

        # The number of step until all agents can be at the workshop
        max_step_count = max([bid.expected_steps for bid in subset])

        if job.end - self._stats_provider.simulation_step < max_step_count + ChooseBestJobCombination.MIN_STEP_BUFFER:
            return 0 # This combination of agents can't make it in time

        activation = max_step_count * ChooseBestJobCombination.WEIGHT_STEPS

        if can_fulfill_job_without_storage_items:
            activation += ChooseBestJobCombination.WEIGHT_NO_STORAGE_NEEDED

        return activation