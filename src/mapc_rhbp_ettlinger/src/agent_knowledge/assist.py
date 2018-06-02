#!/usr/bin/env python2

import rospy
from mapc_rhbp_ettlinger.msg import AssistTask
from mac_ros_bridge.msg import Position

from agent_knowledge.base_knowledge import BaseKnowledgebase


class AssistKnowledgebase(BaseKnowledgebase):

    ASSIST_INDEX_AGENT_NAME = 1
    ASSIST_INDEX_ROLE = 2
    ASSIST_INDEX_LAT = 3
    ASSIST_INDEX_LONG = 4
    ASSIST_INDEX_ASSISTING_AGENT = 5
    ASSIST_INDEX_ACTIVE = 6


    @staticmethod
    def generate_tuple(agent_name="*", role="*", lat="*", long="*", assisting_agent="*", active="*"):
        return 'assist', agent_name, role, str(lat), str(long), assisting_agent, str(active)


    @staticmethod
    def generate_assign_task_from_fact(fact):
        """
        Generates an AssistTask from a fact
        :param fact: The fact
        :type fact: list
        :return: AssistTask
        """
        task = AssistTask(
            agent_name=fact[AssistKnowledgebase.ASSIST_INDEX_AGENT_NAME],
            role=fact[AssistKnowledgebase.ASSIST_INDEX_ROLE],
            pos=Position(
                lat=fact[AssistKnowledgebase.ASSIST_INDEX_LAT],
                long=fact[AssistKnowledgebase.ASSIST_INDEX_LONG]),
            assisting_agent=fact[AssistKnowledgebase.ASSIST_INDEX_ASSISTING_AGENT],
            active=fact[AssistKnowledgebase.ASSIST_INDEX_ACTIVE])
        return task


    @staticmethod
    def generate_fact_from_assign_task(assist_task):
        """
        Generates Fact from  AssistTask
        :param assist_task: The AssistTask
        :type assist_task: AssistTask
        :return: list
        """
        fact = AssistKnowledgebase.generate_tuple(
            agent_name=assist_task.agent_name,
            role=assist_task.role,
            lat=assist_task.pos.lat,
            long=assist_task.pos.long,
            assisting_agent=assist_task.assisting_agent,
            active=assist_task.active)
        return fact

    def request_assist(self, agent_name, role, pos):
        """
        Requests assistance for assembly. This is used temporarily as a substitute for the communication protocol
        :param task:
        :type task: Task
        :return:
        """
        search = AssistKnowledgebase.generate_tuple(
            agent_name=agent_name,
            role=role,
            lat=pos.lat,
            long=pos.long,
        )
        tuple = AssistKnowledgebase.generate_tuple(
            agent_name=agent_name,
            role=role,
            lat=pos.lat,
            long=pos.long,
            assisting_agent="none",
            active=True
        )
        task_tuples = self._kb_client.all(search)

        if len(task_tuples) > 0:
            rospy.loginfo("TaskKnowledge:: Assist request %s:%s already exists", agent_name, role)
            return
        else:
            rospy.loginfo("TaskKnowledge:: Assist request %s:%s saved", agent_name, role)
            self._kb_client.push(tuple)


    def get_assist_task(self, assigned_agent):
        """
        Returns the currently active assist task of an agent
        :param assigned_agent:
        :return: AssistTask
        """
        tuple = self.generate_tuple(
            assisting_agent=assigned_agent,
            active=True)

        fact = self._kb_client.peek(tuple)


        if fact != None:
            task = self.generate_assign_task_from_fact(fact)
            return task

        else:
            return None

    def cancel_assist_requests(self, agent_name):
        """
        Cancels all assist tasks requested by an agent
        :param agent_name: Name of the agent
        :param agent_name: str
        :return:
        """
        all = self.generate_tuple(
            agent_name=agent_name)
        new = self.generate_tuple(
            agent_name=agent_name,
            role="none", lat="0.0",
            long="0.0",
            assisting_agent="none",
            active="false")

        self._kb_client.update(all, new, push_without_existing=False)