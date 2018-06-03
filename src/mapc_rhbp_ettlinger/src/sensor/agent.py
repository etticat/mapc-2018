import rospy
from mac_ros_bridge.msg import Agent

from agent_knowledge.movement import MovementKnowledgebase
from agent_knowledge.resource import ResourceKnowledgebase
from behaviour_components.activators import ThresholdActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition, Negation
from behaviour_components.goals import GoalBase
from behaviour_components.network_behavior import NetworkBehaviour
from behaviour_components.sensors import Sensor
from common_utils.agent_utils import AgentUtils
from provider.product_provider import ProductProvider
from sensor.movement import DestinationDistanceSensor



class StorageAvailableForItemSensor(Sensor):

    def __init__(self,agent_name, behaviour_name, **kwargs):
        super(StorageAvailableForItemSensor, self).__init__(**kwargs)

        self._agent_name = agent_name
        self.free_load = 0
        self._behaviour_name = behaviour_name
        self._resource_knowledgebase = ResourceKnowledgebase()
        self._movement_knowledgebase = MovementKnowledgebase()
        self._product_provider = ProductProvider(agent_name=agent_name)
        rospy.Subscriber(AgentUtils.get_bridge_topic_agent(agent_name=agent_name), Agent, callback=self.callback_agent)

    def callback_agent(self, msg):
        """

        :param self:
        :param msg:
        :type msg: Agent
        :return:
        """

        movement = self._movement_knowledgebase.get_movement(self._agent_name, self._behaviour_name)
        resource = self._resource_knowledgebase.get_resource_by_name(movement.destination)
        if resource != None:
            product = self._product_provider.get_product_by_name(resource.item)
            volume_of_next_gathered_item = product.volume
            item = msg.load_max - msg.load - volume_of_next_gathered_item
            self.update(item)
        else:
            return  0