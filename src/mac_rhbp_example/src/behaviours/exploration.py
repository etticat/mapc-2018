import random

import rospy
from knowledge_base.knowledge_base_client import KnowledgeBaseClient

from agent_knowledge.movement import MovementKnowledge
from behaviour_components.behaviours import BehaviourBase
from behaviours.movement import GotoLocationBehaviour
from utils.ros_helpers import get_topic_type


class ExplorationBehaviour(GotoLocationBehaviour):

    def __init__(self, agent_name, **kwargs):
        super(ExplorationBehaviour, self) \
            .__init__(
            agent_name=agent_name, **kwargs)



        facility_topic = '/shop'


        facility_topic_type = get_topic_type(facility_topic)

        self._facilities = {}
        rospy.Subscriber(facility_topic, facility_topic_type, self._callback_facility)


    def _callback_facility(self, msg):
        # Store all available facilities in a dict
        for facility in msg.facilities:
            self._facilities[facility.name] = facility


    def _select_pos(self):
        l = list(self._facilities.items())
        if(len(l) < 1):
            return False
        _, facility = random.choice(l)

        return facility.pos
        # return Position(lat=48.88958,long=2.30934)

    def _callback_facility(self, msg):
        # Store all available facilities in a dict
        for facility in msg.facilities:
            self._facilities[facility.name] = facility

class FinishExplorationBehaviour(BehaviourBase):
    """
    Behaviour that finishes an exploration cycle by setting a corresponding knowledge fact
    """

    def __init__(self, agent_name, facility_topic, graph_name,  **kwargs):
        """
        Constructor
        """
        super(FinishExplorationBehaviour, self) \
            .__init__(requires_execution_steps=True, **kwargs)

        self._agent_name = agent_name

        self._facility_topic = facility_topic

        self._movement_knowledge = MovementKnowledge(self._agent_name, graph_name)

        self.__client = KnowledgeBaseClient()

    def do_step(self):
        # exploration done
        self._movement_knowledge.stop_movement()

