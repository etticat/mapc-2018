import numpy as np

from diagnostic_msgs.msg import KeyValue
from mapc_rhbp_ettlinger.msg import StockItem, KeyIntValue, TaskStop

import rospy
from mac_ros_bridge.msg import Job, SimStart

from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from common_utils.calc import CalcUtil
from decisions.current_task import CurrentTaskDecision
from my_subscriber import MyPublisher, MySubscriber
from provider.action_provider import ActionProvider, Action
from provider.facility_provider import FacilityProvider
from provider.product_provider import ProductProvider
from provider.simulation_provider import SimulationProvider
from rospy import Publisher

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.decisions.job_activation')


class ChooseBestAvailableJobDecision(object):
    """
    Picks the best available and doable job
    """
    TOPIC_FINISHED_PRODUCT_GOAL = "/planner/job/goals/desired_items"

    DEFAULT_FINISHED_PRODUCT_GOAL = 5
    BID_PERCENTILE = 50
    ACTIVATION_THRESHOLD = -50
    IMPORTANT_JOB_THRESHOLD = 0
    ACTIVATION_TO_DESIRED_PRODUCT_CONVERSION = 0.2
    WEIGHT_PERCENTILE = 10
    WEIGHT_TIME_PASSED = -0.3
    TIME_LEFT_WEIGHT_START = 30  # Steps
    WEIGHT_TIME_OVER = -0.4

    def __init__(self, agent_name):
        self._init_config()

        self.coordinated_jobs = []
        self.active_jobs = []
        self.factors = []

        self._facility_provider = FacilityProvider(agent_name=agent_name)
        self._product_provider = ProductProvider(agent_name=agent_name)
        self._action_provider = ActionProvider(agent_name=agent_name)
        self._simulation_provider = SimulationProvider(agent_name=agent_name)

        MySubscriber(AgentUtils.get_coordination_topic(), message_type="stop",
                     task_type=CurrentTaskDecision.TYPE_DELIVER, callback=self.job_stopped)


        self._pub_desired_finished_products = Publisher(ChooseBestAvailableJobDecision.TOPIC_FINISHED_PRODUCT_GOAL,
                                                        StockItem,
                                                        queue_size=10)
        self._pub_job_stop = MyPublisher(AgentUtils.get_coordination_topic(), message_type="stop",
                                         task_type=CurrentTaskDecision.TYPE_DELIVER, queue_size=10)
        # Reload config after each simulation start
        self._sub_ref = rospy.Subscriber(AgentUtils.get_bridge_topic(agent_name, postfix="start"), SimStart, self._init_config)

    def _init_config(self, sim_start=None):
        ChooseBestAvailableJobDecision.DEFAULT_FINISHED_PRODUCT_GOAL = rospy.get_param(
            "ChooseBestAvailableJobDecision.DEFAULT_FINISHED_PRODUCT_GOAL",
            ChooseBestAvailableJobDecision.DEFAULT_FINISHED_PRODUCT_GOAL)
        ChooseBestAvailableJobDecision.BID_PERCENTILE = rospy.get_param("ChooseBestAvailableJobDecision.BID_PERCENTILE",
                                                                        ChooseBestAvailableJobDecision.BID_PERCENTILE)
        ChooseBestAvailableJobDecision.ACTIVATION_THRESHOLD = rospy.get_param(
            "ChooseBestAvailableJobDecision.ACTIVATION_THRESHOLD",
            ChooseBestAvailableJobDecision.ACTIVATION_THRESHOLD)
        ChooseBestAvailableJobDecision.IMPORTANT_JOB_THRESHOLD = rospy.get_param(
            "ChooseBestAvailableJobDecision.IMPORTANT_JOB_THRESHOLD",
            ChooseBestAvailableJobDecision.IMPORTANT_JOB_THRESHOLD)
        ChooseBestAvailableJobDecision.ACTIVATION_TO_DESIRED_PRODUCT_CONVERSION = rospy.get_param(
            "ChooseBestAvailableJobDecision.ACTIVATION_TO_DESIRED_PRODUCT_CONVERSION",
            ChooseBestAvailableJobDecision.ACTIVATION_TO_DESIRED_PRODUCT_CONVERSION)
        ChooseBestAvailableJobDecision.WEIGHT_PERCENTILE = rospy.get_param(
            "ChooseBestAvailableJobDecision.WEIGHT_PERCENTILE", ChooseBestAvailableJobDecision.WEIGHT_PERCENTILE)
        ChooseBestAvailableJobDecision.WEIGHT_TIME_PASSED = rospy.get_param(
            "ChooseBestAvailableJobDecision.WEIGHT_TIME_PASSED", ChooseBestAvailableJobDecision.WEIGHT_TIME_PASSED)
        ChooseBestAvailableJobDecision.TIME_LEFT_WEIGHT_START = rospy.get_param(
            "ChooseBestAvailableJobDecision.TIME_LEFT_WEIGHT_START",
            ChooseBestAvailableJobDecision.TIME_LEFT_WEIGHT_START)
        ChooseBestAvailableJobDecision.WEIGHT_TIME_OVER = rospy.get_param(
            "ChooseBestAvailableJobDecision.WEIGHT_TIME_OVER", ChooseBestAvailableJobDecision.WEIGHT_TIME_OVER)

    def get_job_activation(self, job, include_fine=True):
        """
        Returns the activation of a job by compring it to previous seen jobs
        :param job:
        :param include_fine:
        :return:
        """
        if include_fine:
            return float(job.reward + job.fine) / self.get_base_ingredient_count(job.items)
        else:
            return float(job.reward) / self.get_base_ingredient_count(job.items)

    def get_base_ingredient_count(self, items):
        """
        Returns the number of gatherable items needed to assemble a finished product
        :param items:
        :return:
        """
        i = 0
        for item in items:
            i += sum(
                self._product_provider.get_gatherable_ingredients_of_product_iteratively(item.name,
                                                                                         item.amount).values())
        return i

    def train_decider(self, job):
        """
        Saves the job activation of each seen job so later jobs can be judged better
        :param job:
        :return:
        """
        # Ignore fee for training
        self.factors.append(self.get_job_activation(job, include_fine=False))

    def choose_job(self):
        """
        Returns the currently available job, where all items are available with the highest activation.
        Also picks the best jobs that we cannot fulfill to decide which items should be prioritised for assembly
        and gathering
        :return:
        """

        # If the factors have just reset, do not coordinatate anything, just wait until the first jobs come in and try again
        if len(self.factors) == 0:
            return None, None

        job_to_try = None
        items_to_take_from_storage = []
        best_activation = ChooseBestAvailableJobDecision.ACTIVATION_THRESHOLD

        # Get all items that agents have available for delivery and that are not already promised to other jobs
        all_available_items = self._product_provider.get_agent_stock_items(types=[
            ProductProvider.STOCK_ITEM_TYPE_STOCK, ProductProvider.STOCK_ITEM_TYPE_DELIVERY_GOAL])

        # create an array of default finished products that is filled later
        desired_finished_product_stock = self.generate_default_desired_finished_product_stock()

        for job in self.active_jobs:

            if job.id in self.coordinated_jobs:
                # Ignore already successfully coordinated job execution
                continue

            # get all job items in a format that is easy to use for later calculations
            job_items = CalcUtil.get_dict_from_items(job.items)

            # Get all items of the target storage
            items_in_destination_storage = self._product_provider.get_stored_items(storage_name=job.storage_name,
                                                                                   include_hoarding_goal=False)
            # Get all items that need to be fulfilled by an agent
            job_items_from_agents = CalcUtil.dict_diff(job_items, items_in_destination_storage, normalize_to_zero=True)
            # Get all items that need to be fulfilled by the target storage
            job_items_from_storage = CalcUtil.dict_diff(job_items, job_items_from_agents)

            job_activation = self.get_job_activation(job=job)

            # get the percentile of the job activation
            percentile = float(sum(i <= job_activation for i in self.factors)) / len(self.factors)

            # Check if all items can be found in agents and storages
            has_all_items = CalcUtil.contains_items(all_available_items, job_items_from_agents)

            time_left = job.end - self._simulation_provider._step
            time_passed = self._simulation_provider._step - job.start

            ettilog.loginfo("%s, ", job.id)
            ettilog.loginfo("->percentile: %f, ", percentile)
            ettilog.loginfo("->reward_quality: %f, ", job_activation)
            ettilog.loginfo("->has_all_items: %s, ", str(has_all_items))
            ettilog.loginfo("->time_left: %f, ", time_left)
            ettilog.loginfo("->time_passed: %f, ", time_passed)

            # Calculate activation of job
            activation = percentile * ChooseBestAvailableJobDecision.WEIGHT_PERCENTILE
            if job.type == "job":
                # The opponent might already do this job, therefore we bias against old jobs
                activation += time_passed * ChooseBestAvailableJobDecision.WEIGHT_TIME_PASSED

            if time_left <= ChooseBestAvailableJobDecision.TIME_LEFT_WEIGHT_START:
                # If less than 30 steps are available start biasing against this job as it will get harder and harder to finish it
                activation += (
                                          ChooseBestAvailableJobDecision.TIME_LEFT_WEIGHT_START - time_left) * ChooseBestAvailableJobDecision.WEIGHT_TIME_OVER

            ettilog.loginfo("activation: %f, ", activation)

            # if all items are available and job is better than all previous ones, pick this job
            if has_all_items and activation > best_activation:
                best_activation = activation
                job_to_try = job
                items_to_take_from_storage = job_items_from_storage

            # if we don't have all items but the job is really good (probably because of the fine), tell the assemblers
            # that the items of this job should be prefered.
            if not has_all_items and activation > ChooseBestAvailableJobDecision.IMPORTANT_JOB_THRESHOLD:
                activation_surpluss = activation - ChooseBestAvailableJobDecision.IMPORTANT_JOB_THRESHOLD
                for item in job.items:
                    desired_finished_product_stock[item.name] = desired_finished_product_stock.get(item.name, 0) + \
                                                                (
                                                                            activation_surpluss * item.amount * ChooseBestAvailableJobDecision.ACTIVATION_TO_DESIRED_PRODUCT_CONVERSION)

        # Publish the desired finished products to all agents
        desired_stock = StockItem(entity="planner")
        for key, value in desired_finished_product_stock.iteritems():
            desired_stock.amounts.append(KeyIntValue(key=key, value=value))
        self._pub_desired_finished_products.publish(desired_stock)

        return job_to_try, items_to_take_from_storage

    def generate_default_desired_finished_product_stock(self):
        """
        The desired finished product stock consists of what the planner wants plus a basic number of all items. This
        function returns the basic item numers
        :return:
        """
        desired_finished_product_stock = {}
        for product in self._product_provider._assemblable_items.keys():
            desired_finished_product_stock[product] = ChooseBestAvailableJobDecision.DEFAULT_FINISHED_PRODUCT_GOAL
        return desired_finished_product_stock

    def get_auction_job_bid(self, job):
        """
        Returns the bid for an auction job
        :param job:
        :return:
        """
        factor = np.percentile(self.factors, ChooseBestAvailableJobDecision.BID_PERCENTILE)

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
            # auction jobs that we won are also included
            if auction.job.start + auction.auction_time <= self._simulation_provider.step:
                extracted_jobs += [auction.job]

        return extracted_jobs

    def save_jobs(self, request_action):
        """
        Keeps track of all jobs. Trains the decider on new jobs and notifies all agents about deleted jobs.
        :param request_action:
        :return:
        """
        # get all jobs from request
        all_jobs_new = self.extract_jobs(request_action)

        # Train decider on all unseen jobs
        for job in all_jobs_new:
            # if job has not been seen before -> process it
            if job not in self.active_jobs:
                # Job is new
                self.train_decider(job)

        # Notify agents about deleted jobs, so they can cancel delivery
        for job in self.active_jobs:
            if job not in all_jobs_new:
                # Job got deleted
                # If job was removed (oponent was quicker, time is up, ...) -> Notify everyone to stop persuing it
                self._pub_job_stop.publish(TaskStop(job_id=job.id, reason="Job was removed"))

        self.active_jobs = all_jobs_new

    def process_auction_jobs(self, auction_jobs):
        """
        Make a bid in the last step
        :param auction_jobs:
        :return:
        """
        for auction in auction_jobs:
            # Only handle the not assigned auctions
            if auction.job.start + auction.auction_time - 1 == self._simulation_provider._step:
                bid = self.get_auction_job_bid(auction.job)

                # Bid for the auction
                if bid > 0:
                    rospy.logerr("ChooseBestAvailableJobDecision:: Bidding for auction_job %s: %d !!!!!!!!!!!!!!!!!!",
                                 auction.job.id, bid)
                    self._action_provider.send_action(action_type=Action.BID_FOR_JOB, params=[
                        KeyValue("Job", str(auction.job.id)),
                        KeyValue("Bid", str(int(bid)))])
                    # We can only bid on one auction each round. Return ...
                    return

    def _on_job_started(self, id):
        """
        When a job is started save this information, so we don't try it again in the near future
        :param id:
        :return:
        """
        self.coordinated_jobs.append(id)

    def job_stopped(self, task_stop):
        """
        When a job is stopped, remove it from coordination. Ths allows that when an agent for whatever reason needs to
        cancel a job that is still active, that another team of agent can start it again.
        :param task_stop:
        :return:
        """
        if task_stop.job_id in self.coordinated_jobs:
            self.coordinated_jobs.remove(task_stop.job_id)

    def reset_decider(self):
        self.factors = []
