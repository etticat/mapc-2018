#!/usr/bin/env python2
from __builtin__ import xrange

import rospy
from knowledge_base.knowledge_base_client import KnowledgeBaseClient
from mac_rhbp_example.msg import AssistTask
from mac_ros_bridge.msg import SimStart, Agent, Position

from agent_common.agent_utils import AgentUtils


class AssistKnowledgebase():

    def __init__(self):
        self.__kb_client = KnowledgeBaseClient(
            knowledge_base_name = "knowledgeBaseNode")

    @staticmethod
    def get_tuple_assist(agent_name="*", role="*", lat="*", long="*", assisting_agent="*", active="*"):
        return 'assist', agent_name, role, str(lat), str(long), assisting_agent, str(active)

    ASSIST_INDEX_AGENT_NAME = 1
    ASSIST_INDEX_ROLE = 2
    ASSIST_INDEX_LAT = 3
    ASSIST_INDEX_LONG = 4
    ASSIST_INDEX_ASSISTING_AGENT = 5
    ASSIST_INDEX_ACTIVE = 6

    def request_assist(self, agent_name, role, pos):
        """
        :param task:
        :type task: Task
        :return:
        """
        search = AssistKnowledgebase.get_tuple_assist(
            agent_name=agent_name,
            role=role,
            lat=pos.lat,
            long=pos.long,
        )
        tuple = AssistKnowledgebase.get_tuple_assist(
            agent_name=agent_name,
            role=role,
            lat=pos.lat,
            long=pos.long,
            assisting_agent="none",
            active=True
        )
        task_tuples = self.__kb_client.all(search)

        if len(task_tuples) > 0:
            rospy.loginfo("TaskKnowledge:: Assist request %s:%s already exists", agent_name, role)
            return
        else:
            rospy.loginfo("TaskKnowledge:: Assist request %s:%s saved", agent_name, role)
            ret_value = self.__kb_client.push(tuple)

    def assign_assist_task(self, task, agent_name):
        # TODO
        pass

    def get_assist_fact(self, assigned_agent):
        """

        :param assigned_agent:
        :return: AssistTask
        """
        tuple = self.get_tuple_assist(
            assisting_agent=assigned_agent,
            active=True)

        facts = self.__kb_client.all(tuple)


        if(len(facts) > 0):
            return AssistTask(
            agent_name=facts[0][AssistKnowledgebase.ASSIST_INDEX_AGENT_NAME],
            role= facts[0][AssistKnowledgebase.ASSIST_INDEX_ROLE],
            pos = Position(
                lat=facts[0][AssistKnowledgebase.ASSIST_INDEX_LAT],
                long=facts[0][AssistKnowledgebase.ASSIST_INDEX_LONG]
            ),
            assisting_agent=facts[0][AssistKnowledgebase.ASSIST_INDEX_ASSISTING_AGENT],
            active = facts[0][AssistKnowledgebase.ASSIST_INDEX_ACTIVE]
        )

        else:
            return None

    def cancel_assist_requests(self, agent_name):
        all = self.get_tuple_assist(agent_name=agent_name)
        new = self.get_tuple_assist(agent_name=agent_name, role="none", lat="0.0", long="0.0", assisting_agent="none", active="false")

        self.__kb_client.update(all,new, push_without_existing=False)