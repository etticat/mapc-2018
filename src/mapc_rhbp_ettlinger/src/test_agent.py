import rospy
from mac_ros_bridge.msg import Position, SimStart

from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from provider.distance_provider import DistanceProvider
from provider.well_provider import WellProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.agent.test')

class TestAgent(object):

    def __init__(self):

        rospy.init_node('planner_node', anonymous=True, log_level=rospy.ERROR)

        self._well_provider = WellProvider()
        self._agent_topic_prefix = AgentUtils.get_bridge_topic_prefix(agent_name="agentA1")

        rospy.Subscriber(self._agent_topic_prefix + "start", SimStart, self._callback_sim_start)


    def _callback_sim_start(self, simStart):
        """

        :param simStart:
        :type simStart: SimStart
        :return:
        """

        pos1 = Position(lat=48.89597, long=2.30791)
        pos2 = Position(lat=48.82903, long=2.28021)

        self.step_provider = DistanceProvider()
        self.step_provider.callback_sim_start(simStart)
        ettilog.logerr("Air distance: %f", self.step_provider.calculate_distance(pos1, pos2))
        ettilog.logerr("Air steps: %f", self.step_provider.calculate_steps(pos1, pos2))
        self.step_provider.can_fly = False
        ettilog.logerr("Street distance: %f", self.step_provider.calculate_distance(pos1, pos2))
        ettilog.logerr("Street steps: %f", self.step_provider.calculate_steps(pos1, pos2))

        rospy.signal_shutdown("test over")



if __name__ == '__main__':

    try:

        # Just take a random agent. doesn't really matter
        job_planner = TestAgent()

        rospy.spin()

    except rospy.ROSInterruptException:
        ettilog.logerr("program interrupted before completion")
