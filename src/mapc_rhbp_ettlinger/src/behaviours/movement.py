#!/usr/bin/env python2


from __future__ import division  # force floating point division when using plain /

import rospy
from diagnostic_msgs.msg import KeyValue
from mac_ros_bridge.msg import GenericAction

from agent_knowledge.task import TaskBaseKnowledge
from behaviour_components.behaviours import BehaviourBase
from behaviours.generic_action import Action
from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.behaviours.movement')


class GoToTaskDestinationBehaviour(BehaviourBase):
    """
    Behaviour that explores the environment by going to a randomly selected facility
    """

    def __init__(self, agent_name, task_type, **kwargs):
        super(GoToTaskDestinationBehaviour, self) \
            .__init__(requires_execution_steps=True, **kwargs)

        self._movement_knowledge = TaskBaseKnowledge()

        self._agent_name = agent_name
        self.task_type = task_type

        self._pub_generic_action = rospy.Publisher(AgentUtils.get_bridge_topic_prefix(agent_name) + 'generic_action',
                                                   GenericAction, queue_size=10)

    @staticmethod
    def action_go_to_location(lat, lon, publisher):
        action = GenericAction()
        action.action_type = Action.GO_TO
        action.params = [
            KeyValue("latitude", str(lat)),
            KeyValue("longitude", str(lon))]
        publisher.publish(action)

    def do_step(self):
        destination = self._movement_knowledge.get_task(agent_name=self._agent_name, type=self.task_type)

        assert destination is not None

        destination = destination.pos

        GoToTaskDestinationBehaviour.action_go_to_location(lat=destination.lat, lon=destination.long,
                                                           publisher=self._pub_generic_action)
