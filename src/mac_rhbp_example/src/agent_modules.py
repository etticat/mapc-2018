from __future__ import division # force floating point division when using plain /
import rospy

import random

from behaviour_components.behaviours import BehaviourBase
from behaviour_components.activators import LinearActivator, PublisherCondition, MultiSensorCondition
from behaviour_components.pddl import Effect

from knowledge_base.knowledge_base_client import KnowledgeBaseClient

from diagnostic_msgs.msg import KeyValue

from mac_ros_bridge.msg import Shop, GenericAction

def get_bridge_topic_prefix(agent_name):
    return 'bridge_node_' + agent_name + '/'

class ExplorationBehaviour(BehaviourBase):
    '''
    Behaviour that explores the environment
    '''

    def __init__(self, agent_name, **kwargs):
        '''
        Constructor
        '''
        super(ExplorationBehaviour, self) \
            .__init__(name="Exploration",
                      requires_execution_steps=True, **kwargs)

        self._agent_name = agent_name

        rospy.Subscriber("/shop", Shop, self._shop_callback)

        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction, queue_size=10)

        self._shops = {}

        self.__client = KnowledgeBaseClient()

    def _shop_callback(self, msg):
        #rospy.loginfo(msg)
        #Store all available shops in a dict
        self._shops[msg.name] = msg

    def move(self):

        name, shops = random.choice(list(self._shops.items()))

        self._action_goto(name)

        self.__client.push((self._agent_name, 'exploring', 'true'))

    def _action_goto(self, name):
        action = GenericAction()
        action.action_type = "goto"
        action.params = [KeyValue("Facility", name)]
        self._pub_generic_action.publish(action)

    def start(self):
        rospy.logdebug("###########" + self._name + " Enabled ###########")

    def stop(self):
        self.__client.push((self._agent_name, 'exploring', 'false'))

    def do_step(self):
        self.move()