#!/usr/bin/env python2
import time
from threading import Thread

from diagnostic_msgs.msg import KeyValue

import rospy
from mac_ros_bridge.msg import RequestAction, SimStart, Position, AuctionJob

from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from contract_net.manager_assemble import AssembleManager
from contract_net.manager_build_well import BuildWellManager
from contract_net.manager_deliver import DeliverManager
from decisions.job_activation import JobDecider
from decisions.well_chooser import ChooseWellToBuild
from provider.action_provider import ActionProvider, Action
from provider.product_provider import ProductProvider
from provider.provider_info_distributor import ProviderInfoDistributor
from provider.simulation_provider import SimulationProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.agent.planner')

class Planner(object):

    def __init__(self, agent_name):

        self.action_provider = ActionProvider(agent_name="agentA1")
        self.simulation_provider = SimulationProvider()
        self._job_decider = JobDecider()
        self._product_provider = ProductProvider(agent_name=agent_name)

        self.well_chooser = ChooseWellToBuild()

        self._agent_topic_prefix = AgentUtils.get_bridge_topic_prefix(agent_name=agent_name)

        self._job_manager = DeliverManager()
        self._build_well_manager = BuildWellManager()
        self._assemble_planner = AssembleManager(agent_name="agentA1")
        self._provider_info_distributor = ProviderInfoDistributor()

        self.coordination_thread = None

        rospy.Subscriber(self._agent_topic_prefix + "request_action", RequestAction, self._action_request_callback)

        rospy.Subscriber(self._agent_topic_prefix + "start", SimStart, self._sim_start_callback)



    def _sim_start_callback(self, sim_start):
        """
        here we could also evaluate the msg in order to initialize depending on the role etc.
        :param sim_start:  the message
        :type sim_start: SimStart
        """

        self._provider_info_distributor.callback_sim_start(sim_start)

        if self.coordination_thread is None:
            self.coordination_thread = Thread(target=self.coordinate, name="coordination")
            self.coordination_thread.start()

    def _action_request_callback(self, request_action):
        """
        here we just trigger the decision-making and plannig
        :param msg: The request for action message
        :type msg: RequestAction
        :return:
        """
        self._provider_info_distributor.callback_request_action(request_action=request_action)

        self.save_jobs(request_action)

    def coordinate(self):

        while self.coordination_thread is not None:
            # self.coordinate_wells()
            self.coordinate_jobs()
            time.sleep(10)
            # self.coordinate_assembly()

    def save_jobs(self, requestAction):
        # get all jobs from request
        all_jobs_new = self.extract_jobs(requestAction)
        self._job_decider.save_jobs(all_jobs_new)

        self.process_auction_jobs(requestAction.auction_jobs)

    def process_auction_jobs(self, auction_jobs):
        for auction in auction_jobs:
            # Only handle the not assigned auctions
            if auction.job.start + auction.auction_time -1 ==self.simulation_provider.step:
                bid = self._job_decider.get_bid(auction.job)
                # rospy.logerr("job: %s", auction.job.id)
                # rospy.logerr("job activation(after acceptance): %f",
                #              self._job_decider.get_job_activation(auction.job, include_fine=True))
                # rospy.logerr("job activation(before acceptance): %f",
                #              self._job_decider.get_job_activation(auction.job, include_fine=False))
                # rospy.logerr("job_reward: %f", auction.job.reward)
                # rospy.logerr("bid: %f", bid)
                # rospy.logerr("job.start: %s", auction.job.start)
                # rospy.logerr("lowest bid: %d", auction.lowest_bid)
                # rospy.logerr("auction time: %d", auction.auction_time)
                # rospy.logerr("max bid: %d", auction.max_bid)
                # rospy.logerr("simulation_step: %d", self.simulation_provider.step)


                # TODO: Move this into agent somehow
                if bid > 0:
                    rospy.logerr("Bidding: %d !!!!!!!!!!!!!!!!!!", bid)
                    self.action_provider.send_action(action_type=Action.BID_FOR_JOB, params=[
                        KeyValue("Job", str(auction.job.id)),
                        KeyValue("Bid", str(int(bid)))])
                else:
                    rospy.logerr("Not Bidding: %d !!!!!!!!!!!!!!!!!!", bid)

    def coordinate_jobs(self):
        job = self._job_decider.job_to_do()

        if job is not None:
            self._job_manager.request_job(job)

    def coordinate_wells(self):
        well_to_build = self.well_chooser.choose_well_type()

        if well_to_build == None:
            return

        pos = Position(lat=0.0, long=0.0)
        self._build_well_manager.build_well(well_to_build, pos)

    def coordinate_assembly(self):

        self._assemble_planner.request_assembly()

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



if __name__ == '__main__':

    rospy.init_node(name='planner', log_level=rospy.INFO)
    try:
        # Just take a random agent. doesn't really matter
        planner = Planner(
            agent_name="agentA1")

        rospy.spin()

    except rospy.ROSInterruptException:
        ettilog.logerr("program interrupted before completion")
