import numpy as np

from diagnostic_msgs.msg import KeyValue
from mapc_rhbp_ettlinger.msg import StockItem

import rospy
from mac_ros_bridge.msg import Job

from common_utils import etti_logging
from common_utils.calc import CalcUtil
from provider.product_provider import ProductProvider
from provider.simulation_provider import SimulationProvider
from rospy import Publisher

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.decisions.job_activation')

class JobDecider(object):
    
    TOPIC_FINISHED_PRODUCT_GOAL = "/planner/job/goals/desired_items"
    BID_PERCENTILE = 50

    def __init__(self):
        self.all_jobs = []
        self._product_provider = ProductProvider(agent_name="agentA1") # TODO: Make independent from agent
        self.simulation_provider = SimulationProvider()
        self.factors = []

        self._pub_desired_finished_products = Publisher(JobDecider.TOPIC_FINISHED_PRODUCT_GOAL, StockItem, queue_size=10)

    def get_job_activation(self, job, include_fine=True):
        if include_fine:
            return float(job.reward + job.fine) / self.get_base_ingredient_count(job.items)
        else:
            return float(job.reward) / self.get_base_ingredient_count(job.items)

    def get_base_ingredient_count(self, items):
        i = 0
        for item in items:
            i += sum(
                self._product_provider.get_ingredients_of_product_iteratively(item.name, item.amount).values())
        return i

    def get_threshold(self):
        percentile = np.percentile(self.factors, 70)
        return percentile

    def train_decider(self, job):
        # Ignore fee for training
        self.factors.append(float(job.reward) / self.get_base_ingredient_count(job.items))
        # TODO: Here we could train a smarter algorithm to learn rm and rs which could later be used for decision making

    def save_jobs(self, all_jobs_new):

        for job in all_jobs_new:
            # if job has not been seen before -> process it
            if job not in self.all_jobs:
                self.train_decider(job)
        self.all_jobs = all_jobs_new

    def job_to_do(self):
        all_available_items = self._product_provider.total_items_in_stock(include_goal=False)

        desired_finished_product_stock = self.generate_default_desired_finished_product_stock()

        ACTIVATION_THRESHOLD = 0
        ACTIVATION_TO_DESIRED_PRODUCT_CONVERSION = 0.2

        job_to_try = None
        best_activation = ACTIVATION_THRESHOLD


        ettilog.logerr("------------------------------------------------")
        for job in self.all_jobs:
            ettilog.loginfo("------------------------------------------------")
            job_items = CalcUtil.get_dict_from_items(job.items)

            reward_quality = self.get_job_activation(job=job)

            percentile = float(sum(i <= reward_quality for i in self.factors)) / len(self.factors)

            has_all_items = CalcUtil.contains_items(all_available_items, job_items)

            time_left = job.end - self.simulation_provider.step
            time_passed = self.simulation_provider.step - job.start

            ettilog.loginfo("%s, ", job.id)
            ettilog.loginfo("->percentile: %f, ", percentile)
            ettilog.loginfo("->reward_quality: %f, ", reward_quality)
            ettilog.loginfo("->has_all_items: %s, ", str(has_all_items))
            ettilog.loginfo("->time_left: %f, ", time_left)
            ettilog.loginfo("->time_passed: %f, ", time_passed)

            WEIGHT_PERCENTILE = 10
            WEIGHT_TIME_PASSED = -0.3

            TIME_LEFT_WEIGHT_START = 30 # Steps

            WEIGHT_TIME_OVER = -0.4

            activation = percentile * WEIGHT_PERCENTILE

            if job.type == "job":
                # The opponent might already do this job, therefore we bias against old jobs
                activation += time_passed * WEIGHT_TIME_PASSED

            if time_left <= TIME_LEFT_WEIGHT_START:
                # If less than 30 steps are available start biasing against this job as it will get harder and harder to finish it
                activation += (TIME_LEFT_WEIGHT_START - time_left) * WEIGHT_TIME_OVER

            ettilog.loginfo("activation: %f, ", activation)

            if has_all_items and activation > best_activation:
                best_activation = activation
                job_to_try = job

            if not has_all_items and activation > ACTIVATION_THRESHOLD:
                activation_surpluss = activation - ACTIVATION_THRESHOLD
                for item in job.items:
                    desired_finished_product_stock[item.name] = desired_finished_product_stock.get(item.name, 0) + \
                                                                (activation_surpluss * item.amount* ACTIVATION_TO_DESIRED_PRODUCT_CONVERSION)


        ettilog.loginfo("------------ %s", str(desired_finished_product_stock))
        ettilog.loginfo("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")

        desired_stock = StockItem(agent="planner")
        for key, value in desired_finished_product_stock.iteritems():
            desired_stock.goals.append(KeyValue(key=key, value=str(value)))

        self._pub_desired_finished_products.publish(desired_stock)
        return job_to_try

    def generate_default_desired_finished_product_stock(self):
        desired_finished_product_stock = {}
        for product in self._product_provider.finished_products.keys():
            desired_finished_product_stock[product] = 5
        return desired_finished_product_stock

    def get_bid(self, job):
        factor = np.percentile(self.factors, JobDecider.BID_PERCENTILE)

        ingredient_count = self.get_base_ingredient_count(job.items)
        return job.reward - factor * ingredient_count


