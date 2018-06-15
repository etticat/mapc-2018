
import rospy
from mapc_rhbp_ettlinger.msg import Task
from mac_ros_bridge.msg import RequestAction, Job, SimStart

from common_utils.agent_utils import AgentUtils
from agent_knowledge.tasks import JobKnowledgebase
from network_coordination.assemble_manager import AssembleManager
from network_coordination.job_manager import JobManager


class AssembleStarter(object):

    def __init__(self):

        rospy.init_node('planner_node', anonymous=True, log_level=rospy.ERROR)

        self.all_jobs = []
        self.all_tasks = []

        self._task_knowledge = JobKnowledgebase()

        self._agent_topic_prefix = AgentUtils.get_bridge_topic_prefix(agent_name="agentA1")

        self._assemble_manager = AssembleManager(agent_name="agentA1")

        rospy.Subscriber(self._agent_topic_prefix + "request_action", RequestAction, self._action_request_callback)


    def _action_request_callback(self, requestAction):
        """
        here we just trigger the decision-making and plannig
        :param msg: The request for action message
        :type msg: RequestAction
        :return:
        """
        self._assemble_manager.request_assist()



if __name__ == '__main__':

    try:

        # Just take a random agent. doesn't really matter
        job_planner = AssembleStarter()

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
