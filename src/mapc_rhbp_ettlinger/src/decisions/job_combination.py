import copy

import itertools

from common_utils import rhbp_logging
from common_utils.calc import CalcUtil
from provider.stats_provider import StatsProvider

ettilog = rhbp_logging.LogManager(logger_name=rhbp_logging.LOGGER_DEFAULT_NAME + '.decisions.job_combination')

class ChooseBestJobCombination(object):

    MIN_STEP_BUFFER = 5

    WEIGHT_LOAD = 8
    WEIGHT_INGREDIENT_LOAD = 8
    WEIGHT_STEPS = -1

    ACTIVATION_THRESHOLD = -20

    def __init__(self):

        self._stats_provider = StatsProvider()

    def choose_best_agent_combination(self, job, bids):

        job_items = CalcUtil.get_list_from_items(job.items)
        best_agent_subset = []
        best_value = -1000

        if len(bids) >= 2:
            # Go through all combinations
            for L in range(1, min(len(bids) + 1, 7)):  # We try all combinations using 1-7 agents
                for subset in itertools.combinations(bids, L):
                    stringi = ""
                    for item in subset:
                        stringi = stringi + item.agent_name + " - "

                    subset_can_fulfill_job = self.job_fulfillment_possible(job_items, subset)

                    if subset_can_fulfill_job:
                        best_value = self.generate_activation(subset, job)
                        best_agent_subset = subset

                if best_value > ChooseBestJobCombination.ACTIVATION_THRESHOLD:
                    # we only try combinations with more, if we could not find anything with less
                    break

        return best_agent_subset


    def job_fulfillment_possible(self, job_items, bids):
        job_items =copy.copy(job_items)

        for bid in bids:

            useful_items = CalcUtil.list_intersect(job_items, bid.items)
            if len(useful_items) > 0:

                job_items = CalcUtil.list_diff(job_items, useful_items)

        return len(job_items) == 0 # If we could not find all items in combination, return empty

    def generate_activation(self, subset, job):

        # The number of step until all agents can be at the workshop
        max_step_count = max([bid.expected_steps for bid in subset])

        if job.end - self._stats_provider.simulation_step < max_step_count + ChooseBestJobCombination.MIN_STEP_BUFFER:
            return 0 # This combination of agents can't make it in time


        return max_step_count * ChooseBestJobCombination.WEIGHT_STEPS