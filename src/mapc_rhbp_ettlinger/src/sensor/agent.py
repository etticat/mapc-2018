import rospy
from mac_ros_bridge.msg import Agent

from behaviour_components.activators import ThresholdActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition, Negation
from behaviour_components.goals import GoalBase
from behaviour_components.network_behavior import NetworkBehaviour
from behaviour_components.sensors import Sensor
from behaviours.job import GoToResourceForHoardingBehaviour, GatherForHoardingBehaviour
from common_utils.agent_utils import AgentUtils
from sensor.movement import DestinationDistanceSensor



class StockStorageAvailableSensor(Sensor):

    def __init__(self,agent_name, **kwargs):
        super(StockStorageAvailableSensor, self).__init__(**kwargs)

        self.free_load = 0
        rospy.Subscriber(AgentUtils.get_bridge_topic_agent(agent_name=agent_name), Agent, callback=self.callback_agent)

    def callback_agent(self, msg):
        """

        :param self:
        :param msg:
        :type msg: Agent
        :return:
        """

        volume_of_next_gathered_item = 5 # TODO: Take correct volume
        rospy.logerr("free load: %s", str(msg.load_max - msg.load - volume_of_next_gathered_item))
        self.update(msg.load_max - msg.load - volume_of_next_gathered_item)
