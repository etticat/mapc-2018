#!/usr/bin/env python2

import rospy
from mapc_rhbp_ettlinger.msg import AssembleTask, AssembleTask
from mac_ros_bridge.msg import Position

from agent_knowledge.base_knowledge import BaseKnowledgebase


class AssembleKnowledgebase(BaseKnowledgebase):

    ASSEMBLE_INDEX_ID = 1
    ASSEMBLE_INDEX_AGENT_NAME = 2
    ASSEMBLE_INDEX_LAT = 3
    ASSEMBLE_INDEX_LONG = 4
    ASSEMBLE_INDEX_TASKS = 5
    ASSEMBLE_INDEX_ACTIVE = 6


    @staticmethod
    def generate_tuple(id="*", agent_name="*", lat="*", long="*", tasks="*", active="*"):
        return 'assemble', id, agent_name, str(lat), str(long), tasks, str(active)


    @staticmethod
    def generate_assemble_task_from_fact(fact):
        """
        Generates an AssembleTask from a fact
        :param fact: The fact
        :type fact: list
        :return: AssembleTask
        """
        task = AssembleTask(
            id=fact[AssembleKnowledgebase.ASSEMBLE_INDEX_ID],
            agent_name=fact[AssembleKnowledgebase.ASSEMBLE_INDEX_AGENT_NAME],
            pos=Position(
                lat=fact[AssembleKnowledgebase.ASSEMBLE_INDEX_LAT],
                long=fact[AssembleKnowledgebase.ASSEMBLE_INDEX_LONG]),
            tasks=fact[AssembleKnowledgebase.ASSEMBLE_INDEX_TASKS],
            active=bool(fact[AssembleKnowledgebase.ASSEMBLE_INDEX_ACTIVE])
        )
        return task


    @staticmethod
    def generate_fact_from_assemble_task(assemble_task):
        """
        Generates Fact from  AssembleTask
        :param assemble_task: The AssembleTask
        :type assemble_task: AssembleTask
        :return: list
        """
        fact = AssembleKnowledgebase.generate_tuple(
            agent_name=assemble_task.agent_name,
            id=assemble_task.id,
            lat=assemble_task.pos.lat,
            long=assemble_task.pos.long,
            tasks=assemble_task.tasks,
            active=assemble_task.active,
        )
        return fact

    def save_assemble(self, task):
        """
        Requests assembleance for assembly. This is used temporarily as a substitute for the communication protocol
        :param task:
        :type task: Task
        :return:
        """
        search = AssembleKnowledgebase.generate_tuple(
            agent_name=task.agent_name,
            active=True
        )
        task_tuples = self._kb_client.all(search)

        if len(task_tuples) > 0:
            rospy.logerr("TaskKnowledge:: Agent %s is already busy", task.agent_name)
            return False
        else:

            search = AssembleKnowledgebase.generate_tuple(
                agent_name=task.agent_name
            )
            tuple = AssembleKnowledgebase.generate_fact_from_assemble_task(task)
            rospy.loginfo("TaskKnowledge:: Agent assembly task saved for %s", task.agent_name)
            self._kb_client.update(search, tuple, push_without_existing=True)
            return True


    def get_assemble_task(self, agent):
        """
        Returns the currently active assemble task of an agent
        :param agent:
        :return: AssembleTask
        """
        tuple = self.generate_tuple(
            agent_name=agent,
            active=True)

        fact = self._kb_client.peek(tuple)


        if fact != None:
            task = self.generate_assemble_task_from_fact(fact)
            return task

        else:
            return None

    def cancel_assemble_requests(self, id):
        """
        Cancels all assemble tasks requested by an agent
        :param agent_name: Name of the agent
        :param agent_name: str
        :return:
        """
        tuple = self.generate_tuple(
            id=id)

        cancelled = self._kb_client.pop(tuple)
