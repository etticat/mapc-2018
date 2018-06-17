import time

import rospy
from mac_ros_bridge.msg import Item, Position, Job, RequestAction, SimStart

from common_utils.agent_utils import AgentUtils
from network_coordination.assemble_contractor import AssembleContractor
from network_coordination.assemble_manager import AssembleManager
from network_coordination.job_contractor import JobContractor
from network_coordination.job_manager import JobManager
from provider.product_provider import ProductProvider
from provider.well_provider import WellProvider


class TestAgent(object):

    def __init__(self):

        rospy.init_node('planner_node', anonymous=True, log_level=rospy.ERROR)

        self._well_provider = WellProvider()
        self._agent_topic_prefix = AgentUtils.get_bridge_topic_prefix(agent_name="agentA1")

        rospy.Subscriber(self._agent_topic_prefix + "request_action", RequestAction, self._callback_action_request)
        rospy.Subscriber(self._agent_topic_prefix + "start", SimStart, self._callback_sim_start)


    def _callback_sim_start(self, simStart):
        """

        :param simStart:
        :type simStart: SimStart
        :return:
        """

    def _callback_action_request(self, requestAction):
        """

        :param requestAction:
        :type requestAction: RequestAction
        :return:
        """
        rospy.logerr("Wells: %s", self._well_provider.wells.keys())
        rospy.logerr("Wells to build: %s", self._well_provider.wells_to_build.keys())




if __name__ == '__main__':

    try:

        # Just take a random agent. doesn't really matter
        job_planner = TestAgent()

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
