#!/usr/bin/env python2


from __future__ import division  # force floating point division when using plain /

import rospy
from diagnostic_msgs.msg import KeyValue
from mac_ros_bridge.msg import GenericAction, Agent

from agent_knowledge.task import TaskKnowledgeBase
from behaviour_components.behaviours import BehaviourBase
from behaviours.generic_action import Action
from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.behaviours.movement')


class GoToTaskDestinationBehaviour(BehaviourBase):
    """
    Behaviour that explores the environment by going to a randomly selected facility
    """

    def __init__(self, agent_name, task_type, cancel_task_on_no_route=False, **kwargs):
        super(GoToTaskDestinationBehaviour, self) \
            .__init__(requires_execution_steps=True, **kwargs)

        self._task_knowledge_base = TaskKnowledgeBase()

        self._agent_name = agent_name
        self.task_type = task_type
        self.cancel_task_on_no_route = cancel_task_on_no_route

        self._pub_generic_action = rospy.Publisher(AgentUtils.get_bridge_topic_prefix(agent_name) + 'generic_action',
                                                   GenericAction, queue_size=10)

        rospy.Subscriber(AgentUtils.get_bridge_topic_prefix(agent_name=self._agent_name) + "agent", Agent,
                         self._action_request_agent)

    def _action_request_agent(self, agent):
        """

        :param agent:
        :type agent: Agent
        :return:
        """
        # Sometimes movement doesnt work (e.g. when navigating into a building. just cancel ..
        if agent.last_action == "goto" and agent.last_action_result == "failed_no_route" and self.cancel_task_on_no_route:
            self._task_knowledge_base.finish_task(agent_name=self._agent_name, type=self.task_type)

    @staticmethod
    def action_go_to_location(lat, lon, publisher):
        action = GenericAction()
        action.action_type = Action.GO_TO
        action.params = [
            KeyValue("latitude", str(lat)),
            KeyValue("longitude", str(lon))]
        publisher.publish(action)

    def do_step(self):
        destination = self._task_knowledge_base.get_task(agent_name=self._agent_name, type=self.task_type)

        assert destination is not None

        destination = destination.pos

        if self._name == "go_to_resource_node_behaviour":
                ettilog.logerr("(%s) Go to location: %s", self._agent_name, str(destination))

        GoToTaskDestinationBehaviour.action_go_to_location(lat=destination.lat, lon=destination.long,
                                                           publisher=self._pub_generic_action)
