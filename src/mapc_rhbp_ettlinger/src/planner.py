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

        self._job_decider.save_jobs(request_action)

    def coordinate(self):

        while self.coordination_thread is not None:
            # self.coordinate_wells()
            self.coordinate_jobs()
            time.sleep(10)
            # self.coordinate_assembly()

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



if __name__ == '__main__':

    rospy.init_node(name='planner', log_level=rospy.INFO)
    try:
        # Just take a random agent. doesn't really matter
        planner = Planner(
            agent_name="agentA1")

        rospy.spin()

    except rospy.ROSInterruptException:
        ettilog.logerr("program interrupted before completion")
