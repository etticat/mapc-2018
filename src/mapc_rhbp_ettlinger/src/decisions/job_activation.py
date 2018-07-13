import numpy as np

from diagnostic_msgs.msg import KeyValue
from mapc_rhbp_ettlinger.msg import StockItem, KeyIntValue, TaskStop

import rospy
from mac_ros_bridge.msg import Job

from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from common_utils.calc import CalcUtil
from decisions.p_task_decision import CurrentTaskDecision
from provider.action_provider import ActionProvider, Action
from provider.facility_provider import FacilityProvider
from provider.product_provider import ProductProvider
from provider.simulation_provider import SimulationProvider
from rospy import Publisher
from rospy.my_publish_subscribe import MyPublisher

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.decisions.job_activation')

class JobDecider(object):
    
    DEFAULT_FINISHED_PRODUCT_GOAL = 5
    TOPIC_FINISHED_PRODUCT_GOAL = "/planner/job/goals/desired_items"
    BID_PERCENTILE = 50

    ACTIVATION_THRESHOLD = 0
    ACTIVATION_TO_DESIRED_PRODUCT_CONVERSION = 0.2

    WEIGHT_PERCENTILE = 10
    WEIGHT_TIME_PASSED = -0.3

    TIME_LEFT_WEIGHT_START = 30  # Steps

    WEIGHT_TIME_OVER = -0.4

    def __init__(self, agent_name="agentA1"):
        self.facility_provider = FacilityProvider()
        self.coordinated_jobs = []
        self.active_jobs = []
        self._product_provider = ProductProvider(agent_name=agent_name) # TODO: Make independent from agent
        self.action_provider = ActionProvider(agent_name=agent_name)
        self.simulation_provider = SimulationProvider()
        self.factors = []

        self._pub_desired_finished_products = Publisher(JobDecider.TOPIC_FINISHED_PRODUCT_GOAL, StockItem, queue_size=10)

        self._pub_job_stop = MyPublisher(AgentUtils.get_coordination_topic(), message_type="stop", task_type=CurrentTaskDecision.TYPE_DELIVER, queue_size=10)

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
        self.factors.append(self.get_job_activation(job, include_fine=False))

    def job_to_do(self):
        all_available_items = self._product_provider.total_items_in_stock(types=ProductProvider.STOCK_ITEM_ALL_AGENT_TYPES)

        desired_finished_product_stock = self.generate_default_desired_finished_product_stock()

        job_to_try = None
        items_to_take_from_storage = []
        best_activation = JobDecider.ACTIVATION_THRESHOLD


        for job in self.active_jobs:

            if job.id in self.coordinated_jobs:
                # Already successfully coordinated job execution  -> ignore
                continue

            job_items = CalcUtil.get_dict_from_items(job.items)

            items_in_destination_storage = self._product_provider.get_stored_items(storage_name=job.storage_name)
            job_items_from_agents = CalcUtil.dict_diff(job_items, items_in_destination_storage, normalize_to_zero=True)
            job_items_from_storage = CalcUtil.dict_diff(job_items, job_items_from_agents)

            reward_quality = self.get_job_activation(job=job)

            percentile = float(sum(i <= reward_quality for i in self.factors)) / len(self.factors)

            has_all_items = CalcUtil.contains_items(all_available_items, job_items_from_agents)

            time_left = job.end - self.simulation_provider.step
            time_passed = self.simulation_provider.step - job.start

            ettilog.loginfo("%s, ", job.id)
            ettilog.loginfo("->percentile: %f, ", percentile)
            ettilog.loginfo("->reward_quality: %f, ", reward_quality)
            ettilog.loginfo("->has_all_items: %s, ", str(has_all_items))
            ettilog.loginfo("->time_left: %f, ", time_left)
            ettilog.loginfo("->time_passed: %f, ", time_passed)


            activation = percentile * JobDecider.WEIGHT_PERCENTILE

            if job.type == "job":
                # The opponent might already do this job, therefore we bias against old jobs
                activation += time_passed * JobDecider.WEIGHT_TIME_PASSED

            if time_left <= JobDecider.TIME_LEFT_WEIGHT_START:
                # If less than 30 steps are available start biasing against this job as it will get harder and harder to finish it
                activation += (JobDecider.TIME_LEFT_WEIGHT_START - time_left) * JobDecider.WEIGHT_TIME_OVER

            ettilog.loginfo("activation: %f, ", activation)

            if has_all_items and activation > best_activation:
                best_activation = activation
                job_to_try = job
                items_to_take_from_storage = job_items_from_storage

            if not has_all_items and activation > JobDecider.ACTIVATION_THRESHOLD:
                activation_surpluss = activation - JobDecider.ACTIVATION_THRESHOLD
                for item in job.items:
                    desired_finished_product_stock[item.name] = desired_finished_product_stock.get(item.name, 0) + \
                                                                (activation_surpluss * item.amount* JobDecider.ACTIVATION_TO_DESIRED_PRODUCT_CONVERSION)


        desired_stock = StockItem(entity="planner")
        for key, value in desired_finished_product_stock.iteritems():
            desired_stock.amounts.append(KeyIntValue(key=key, value=value))

        self._pub_desired_finished_products.publish(desired_stock)
        return job_to_try, items_to_take_from_storage

    def generate_default_desired_finished_product_stock(self):
        desired_finished_product_stock = {}
        for product in self._product_provider.finished_products.keys():
            desired_finished_product_stock[product] = JobDecider.DEFAULT_FINISHED_PRODUCT_GOAL
        return desired_finished_product_stock

    def get_bid(self, job):
        factor = np.percentile(self.factors, JobDecider.BID_PERCENTILE)

        ingredient_count = self.get_base_ingredient_count(job.items)
        return job.reward - factor * ingredient_count

    def extract_jobs(self, msg):
        """
        Extracts all jobs from the RequestAction into a list
        :param msg: The request for action message
        :type msg: RequestAction
        :return:
        """

        extracted_jobs = []

        for mission in msg.mission_jobs:
            extracted_jobs += [mission]
        for priced in msg.priced_jobs:
            extracted_jobs += [priced]
        for auction in msg.auction_jobs:
            if auction.job.start + auction.auction_time <= self.simulation_provider.step:
                extracted_jobs += [auction.job]

        return extracted_jobs

    def save_jobs(self, requestAction):
        # get all jobs from request
        all_jobs_new = self.extract_jobs(requestAction)

        for job in all_jobs_new:
            # if job has not been seen before -> process it
            if job not in self.active_jobs:
                # Job is new
                self.train_decider(job)

        for job in self.active_jobs:
            if job not in all_jobs_new:
                # Job got deleted
                # If job was removed (oponent was quicker, time is up, ...) -> Notify everyone to stop persuing it
                self._pub_job_stop.publish(TaskStop(job_id=job.id, reason="Job was removed"))

        self.active_jobs = all_jobs_new

    def process_auction_jobs(self, auction_jobs):
        for auction in auction_jobs:
            # Only handle the not assigned auctions
            if auction.job.start + auction.auction_time -1 == self.simulation_provider.step:
                bid = self.get_bid(auction.job)

                if bid > 0:
                    rospy.logerr("Bidding: %d !!!!!!!!!!!!!!!!!!", bid)
                    self.action_provider.send_action(action_type=Action.BID_FOR_JOB, params=[
                        KeyValue("Job", str(auction.job.id)),
                        KeyValue("Bid", str(int(bid)))])
                    # We can only bid on one auction each round. Return ...
                    return

