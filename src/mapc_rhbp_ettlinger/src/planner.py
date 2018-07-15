#!/usr/bin/env python2
from threading import Thread

import rospy
from mac_ros_bridge.msg import RequestAction, SimStart, Position

from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from common_utils.calc import CalcUtil
from contract_net.manager_assemble import AssembleManager
from contract_net.manager_build_well import BuildWellManager
from contract_net.manager_deliver import DeliverManager
from decisions.choose_best_available_job import ChooseBestAvailableJobDecision
from decisions.well_chooser import ChooseWellToBuildDecision
from provider.product_provider import ProductProvider
from provider.simulation_provider import SimulationProvider
from global_rhbp_components import GlobalRhbpComponents

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.node.planner')


class Planner(object):
    """
    Central planner, that keeps track of all jobs and distributes the best ones. Also responsible for Assembly coordination
    TODO: Well coordination
    """

    def __init__(self):

        rospy.init_node(name='planner', log_level=rospy.ERROR)
        agent_name = rospy.get_param('~agent_name', "agentA1")

        # Initialise providers
        self._simulation_provider = SimulationProvider(agent_name=agent_name)
        self._product_provider = ProductProvider(agent_name=agent_name)

        # Initialise mechanisms
        self._job_decider = ChooseBestAvailableJobDecision(agent_name=agent_name)
        self.well_chooser = ChooseWellToBuildDecision(agent_name=agent_name)

        # Initialise rhbp components
        self._global_rhbp_components = GlobalRhbpComponents(agent_name=agent_name)

        # Init contract net managers
        self._manager_deliver = DeliverManager(agent_name=agent_name)
        self._manager_well = BuildWellManager(self.well_chooser)
        self._manager_assemble = AssembleManager(agent_name=agent_name,
                                                 assembly_combination_decision=self._global_rhbp_components.assembly_combination_decision)

        # Keep reference to coordination thread
        self.coordination_thread = None

        # Register to topics
        self._agent_topic_prefix = AgentUtils.get_bridge_topic_prefix(agent_name=agent_name)
        rospy.Subscriber(self._agent_topic_prefix + "request_action", RequestAction, self._callback_action_request)
        rospy.Subscriber(self._agent_topic_prefix + "start", SimStart, self._sim_start_callback)

    def _sim_start_callback(self, sim_start):
        """
        When the simulation is started, start the coordination thread
        :param sim_start:  the message
        :type sim_start: SimStart
        """

        if self.coordination_thread is None:
            self.coordination_thread = Thread(target=self.coordinate, name="coordination")
            self.coordination_thread.start()

    def _callback_action_request(self, request_action):
        """
        here we just trigger the decision-making and plannig
        :param msg: The request for action message
        :type msg: RequestAction
        :return:
        """
        self._job_decider.save_jobs(request_action)

    def coordinate(self):
        """
        Coordinates delivery jobs, assembly jobs and well building.
        Warning: Blocking. Has to be run in seperate thread
        :return: None
        """

        while self.coordination_thread is not None:
            # self.coordinate_wells()
            self.coordinate_jobs()
            self.coordinate_assembly()

    def coordinate_jobs(self):
        """
        Tries to coordinate the best job, that is currently available
        :return:
        """
        # Pick the best job, that currently seems to be possible
        job, items_to_pickup = self._job_decider.choose_job()

        # If job is found -> Try to perform it
        if job is not None:
            ettilog.logerr("Planner: decided for job: %s agent_items: %s, storage_items %s", job.id,
                           CalcUtil.get_dict_from_items(job.items), items_to_pickup)
            # Try to find agents to perform job using contract net. This may take a few seconds
            successful = self._manager_deliver.request_job(job)
            if successful:
                # If distribution was successful, inform the job decider
                self._job_decider._on_job_started(job.id)

    def coordinate_wells(self):
        """
        Coordinates Well building. If enough mssim is available to build a well, let a random agent build it.
        TODO: This should be moved into the agent itself (Maybe throgh self organisation?)
        :return:
        """

        #Choose well type that we can currently afford with massium
        well_to_build = self.well_chooser.choose_well_type()

        if well_to_build is not None:
            # If we found a well type to build, assign an agent. This may take a few seconds
            self._manager_well.build_well(well_to_build, pos=Position(lat=0.0, long=0.0))

    def coordinate_assembly(self):
        """
        Tries to coordinate assembly.
        :return: None
        """
        # This may take a few seconds
        self._manager_assemble.request_assembly()


if __name__ == '__main__':

    try:
        planner = Planner()
        rospy.spin()

    except rospy.ROSInterruptException:
        ettilog.logerr("program interrupted before completion")
