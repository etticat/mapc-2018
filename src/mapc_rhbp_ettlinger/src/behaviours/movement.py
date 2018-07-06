#!/usr/bin/env python2


from __future__ import division  # force floating point division when using plain /

import rospy
from diagnostic_msgs.msg import KeyValue
from mac_ros_bridge.msg import GenericAction, Agent

from agent_knowledge.task import TaskKnowledgeBase
from behaviour_components.behaviours import BehaviourBase
from provider.action_provider import Action
from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from provider.action_provider import ActionProvider

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
        self.action_provider = ActionProvider(agent_name=agent_name)

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

    def do_step(self):
        destination = self._task_knowledge_base.get_task(agent_name=self._agent_name, type=self.task_type)

        assert destination is not None

        destination = destination.pos

        self.action_provider.action_go_to_location(lat=destination.lat, lon=destination.long)
