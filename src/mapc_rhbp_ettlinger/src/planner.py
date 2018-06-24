#!/usr/bin/env python2
import rospy
from mac_ros_bridge.msg import RequestAction, SimStart

from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from coordination.assemble_manager import AssembleManager
from coordination.build_well_manager import BuildWellManager
from coordination.job_manager import JobManager
from decisions.job_activation import JobDecider
from decisions.well_chooser import ChooseWellToBuild
from provider.product_provider import ProductProvider
from provider.provider_info_distributor import ProviderInfoDistributor

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.agent.planner')

class Planner(object):

    def __init__(self, agent_name):

        self._job_decider = JobDecider()
        self._product_provider = ProductProvider(agent_name=agent_name)

        self.all_jobs = []
        self.all_tasks = []

        self.well_chooser = ChooseWellToBuild()

        self._agent_topic_prefix = AgentUtils.get_bridge_topic_prefix(agent_name=agent_name)

        self.job_manager = JobManager()
        self.well_manager = BuildWellManager()
        self._assemble_planner = AssembleManager(agent_name="agentA1")
        self._provider_info_distributor = ProviderInfoDistributor()

        rospy.Subscriber(self._agent_topic_prefix + "request_action", RequestAction, self._action_request_callback)

        rospy.Subscriber(self._agent_topic_prefix + "start", SimStart, self._sim_start_callback)


    def _sim_start_callback(self, sim_start):
        """
        here we could also evaluate the msg in order to initialize depending on the role etc.
        :param sim_start:  the message
        :type sim_start: SimStart
        """

        self._provider_info_distributor.callback_sim_start(sim_start)

    def _action_request_callback(self, request_action):
        """
        here we just trigger the decision-making and plannig
        :param msg: The request for action message
        :type msg: RequestAction
        :return:
        """
        if self.job_manager.busy:
            ettilog.loginfo("JobPlanner:: Job Manager busy. Skiping step ....")
            return

        self._provider_info_distributor.callback_request_action(request_action=request_action)

        self.coordinate_wells(request_action)
        self.coordinate_jobs(request_action)
        self.coordinate_assembly(request_action)


        ettilog.loginfo("JobPlanner:: Jobs processed")

    def coordinate_jobs(self, requestAction):
        # get all jobs from request
        all_jobs_new = self.extract_jobs(requestAction)
        for job in all_jobs_new:
            # if job has not been seen before -> process it
            if job not in self.all_jobs:
                # ettilog.logerr("JobPlanner:: processing new job %s, %d: %d factor: %f", job.id, interna_value, job.reward, (job.reward/float(interna_value)))

                self._job_decider.train_decider(job)
                job_activation = self._job_decider.get_job_activation(job)
                if job_activation > self._job_decider.get_threshold():
                    ettilog.loginfo("job: %s, activation: %f, type: %s, items: %s", job.id, job_activation, job.type,
                                  str([item.name + " (" + str(item.amount) + ") " for item in job.items]))
                    self.job_manager.job_request(job)
        self.all_jobs = all_jobs_new

    def coordinate_wells(self, msg):
        well_to_build = self.well_chooser.choose_well_type()

        if well_to_build == None:
            return

        self.well_manager.well_request(well_to_build, self.well_chooser.choose_well_position())

    def coordinate_assembly(self, requestAction):

        self._assemble_planner.request_assist()

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
        #for auction in msg.auction_jobs:
        #    all_jobs_new.add(auction.job)
        for priced in msg.priced_jobs:
            extracted_jobs += [priced]

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
