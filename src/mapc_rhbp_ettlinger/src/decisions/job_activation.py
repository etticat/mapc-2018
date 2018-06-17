import numpy as np
import rospy

from provider.product_provider import ProductProvider


class JobDecider(object):

    def __init__(self):
        self._product_provider = ProductProvider(agent_name="agentA1") # TODO: Make independent from agent
        self.factors = []

    def get_job_activation(self, job):
        return float(job.reward + job.fine) / self.get_base_ingredient_count(job.items)

    def get_base_ingredient_count(self, items):
        i = 0
        for item in items:
            i += sum(
                self._product_provider.get_base_ingredients_of_product_iteratively(item.name, item.amount).values())
        return i

    def get_threshold(self):
        percentile = np.percentile(self.factors, 70)
        return percentile

    def train_decider(self, job):
        # Ignore fee for training
        self.factors.append(float(job.reward) / self.get_base_ingredient_count(job.items))
        # TODO: Here we could train a smarter algorithm to learn rm and rs which could later be used for decision making