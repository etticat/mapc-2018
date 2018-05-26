#!/usr/bin/env python2

import rospy
from knowledge_base.knowledge_base_client import KnowledgeBaseClient
from mac_rhbp_example.msg import Task
from mac_ros_bridge.msg import SimStart

from agent_common.agent_utils import AgentUtils


class TaskKnowledge():

    def __init__(self):
        self.__kb_client = KnowledgeBaseClient(
            knowledge_base_name = "knowledgeBaseNode")

        self.products = {}

        # TODO: When I call this method, the whole program crashes without error
        # rospy.Subscriber(
        #     AgentUtils.get_bridge_topic_agent(agent_name=agent_name),
        #     SimStart,
        #     self._callback_sim_start)

    @staticmethod
    def get_tuple_task(job_id, task_id):
        return 'task', job_id, task_id
    
    @staticmethod
    def get_tuple_task_creation(job_id, task_id, destination, agent, status):
        return 'task', job_id, task_id, destination, agent, status

    def _callback_sim_start(self, msg):
        """

        :param msg:
        :type msg: SimStart
        :return:
        """
        for product in msg.products:
            self.products[product.name] = product


    def save_task(self, task):
        """
        :param task:
        :type task: Task
        :return:
        """
        tuple = TaskKnowledge.get_tuple_task_creation(
            job_id=task.job_id,
            task_id=task.id,
            destination="*",
            agent="*",
            status="*")
        task_tuples = self.__kb_client.all(tuple)

        if len(task_tuples) > 0:
            rospy.logerr("TaskKnowledge:: Task %s%s already exists", task.job_id, task.id)
            return
        else:
            rospy.logerr("TaskKnowledge:: Task %s%s saved", task.job_id, task.id)
            new = TaskKnowledge.get_tuple_task_creation(job_id=task.job_id, task_id=task.id, destination=task.destination, agent="agentA1", status="assigned")
            ret_value = self.__kb_client.push(new)

    def assign_task(self, task, agent_name):
        search = TaskKnowledge.get_tuple_task_creation(job_id=task.job_id, task_id=task.id, destination=task.destination, agent="none", status="open")
        new = TaskKnowledge.get_tuple_task_creation(job_id=task.job_id, task_id=task.id, destination=task.destination, agent="none", status="assigned")

        ret_value = self.__kb_client.update(search, new, push_without_existing = True)
        return ret_value

    def finish_task(self, task, agent_name):
        search = TaskKnowledge.get_tuple_task_creation(job_id=task.job_id, task_id=task.id, destination=task.destination, agent="none", status="assigned")
        new = TaskKnowledge.get_tuple_task_creation(job_id=task.job_id, task_id=task.id, destination=task.destination, agent="none", status="finished")

        ret_value = self.__kb_client.update(search, new, push_without_existing = True)
        return ret_value

    def get_task(self, agent_name, status="*", job_id="*", destination="*", task_id="*"):
        tuple = TaskKnowledge.get_tuple_task_creation(
            job_id=job_id,
            task_id=task_id,
            destination=destination,
            agent=agent_name,
            status=status)
        task_tuple = self.__kb_client.peek(tuple)

        task = Task(
            job_id=task_tuple[1],
            id=task_tuple[2],
            destination=task_tuple[3],
            agent=task_tuple[4],
            status=task_tuple[5])

        return task
