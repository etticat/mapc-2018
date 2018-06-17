import copy
import itertools

from mapc_rhbp_ettlinger.msg import JobAssignment

from common_utils.calc import CalcUtil


class ChooseBestJobCombination(object):

    WEIGHT_LOAD = 8
    WEIGHT_INGREDIENT_LOAD = 8
    WEIGHT_STEPS = -9

    ACTIVATION_THRESHOLD = 77

    def __init__(self):

        self.max_load = 100
        self.load = 60
        self.load_ingredients = 40
        self.load_finished_products = 35
        self.speed = 10
        self.transport = "air"
        self.items = {}

    def choose(self, job, bids):

        job_items = CalcUtil.get_list_from_items(job.items)
        best_combination = []
        best_value = 0
        best_finished_products = None

        if len(bids) >= 2:
            # Go through all combinations
            for L in range(1, min(len(bids) + 1, 7)):  # We try all combinations using 1-7 agents
                for subset in itertools.combinations(bids, L):
                    stringi = ""
                    for item in subset:
                        stringi = stringi + item.agent_name + " - "

                    combination = self.generate_job_fulfillment_combination(job_items, subset)

                    if combination != None:
                        best_value = 1
                        best_combination = subset
                        best_finished_products = combination
                if best_value > 0:
                    # we only try combinations with more, if we could not find anything with less
                    break

        return best_finished_products


    def generate_job_fulfillment_combination(self, job_items, bids):
        res = []
        job_items =copy.copy(job_items)

        for bid in bids:

            useful_items = CalcUtil.list_intersect(job_items, bid.items)
            if len(useful_items) >  0:
                assignement = JobAssignment(
                    id=bid.id,
                    agent_name=bid.agent_name,
                    assigned=True,
                    deadline=0,
                    items=useful_items)
                job_items = CalcUtil.list_diff(job_items, useful_items)
                res.append(assignement)

        if len(job_items) > 0: # If we could not find all items in combination, return empty
            return None

        return res

