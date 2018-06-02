#!/usr/bin/env python2

import rospy
from mac_rhbp_example.msg import Task

from agent_knowledge.base_knowledge import BaseKnowledgebase

class TaskKnowledgebase(BaseKnowledgebase):

    INDEX_JOB_ID = 1
    INDEX_TASK_ID = 2
    INDEX_DESTINATION = 3
    INDEX_AGENT_NAME = 4
    INDEX_STATUS = 5
    INDEX_ITEM = 6


    @staticmethod
    def generate_tuple(job_id="*", task_id="*", destination="*", agent="*", status="*", item="*"):
        return 'task', job_id, task_id, destination, agent, status, item

    @staticmethod
    def generate_task_from_fact(fact):
        """
        Generates a Task from a Knowledgebase fact
        :param fact: The fact
        :type fact: list
        :return: Task
        """
        task = Task(
            job_id = fact[TaskKnowledgebase.INDEX_JOB_ID],
            id = fact[TaskKnowledgebase.INDEX_TASK_ID],
            destination = fact[TaskKnowledgebase.INDEX_DESTINATION],
            item = fact[TaskKnowledgebase.INDEX_ITEM],
            agent = fact[TaskKnowledgebase.INDEX_AGENT_NAME],
            status = fact[TaskKnowledgebase.INDEX_STATUS],
        )
        return task

    @staticmethod
    def generate_fact_from_task(task):
        """
        Generates a knowledgebase fact rom a task
        :param task: The task
        :type task: Task
        :return: list
        """
        return TaskKnowledgebase.generate_tuple(
            job_id=task.job_id,
            task_id=task.id,
            destination=task.destination,
            item=task.item,
            agent=task.agent,
            status=task.status

        )

    def save_task(self, task):
        """
        Saves a new task to the Knoledgebase
        :param task:
        :type task: Task
        :return:
        """
        tuple = TaskKnowledgebase.generate_tuple(
            job_id=task.job_id,
            task_id=task.id,
            destination="*",
            agent="*",
            status="*",
            item=task.item
        )
        task_tuples = self._kb_client.all(tuple)

        if len(task_tuples) > 0:
            rospy.loginfo("TaskKnowledge:: Task %s%s already exists", task.job_id, task.id)
            return
        else:
            new = TaskKnowledgebase.generate_tuple(job_id=task.job_id, task_id=task.id, destination=task.destination, agent="none", status="none", item=task.item)
            rospy.loginfo("TaskKnowledge:: Task %s%s saved", task.job_id, task.id)
            ret_value = self._kb_client.push(new)

    def assign_task(self, task, agent_name):
        """
        Assigns a task to an agent
        :param task: The task
        :type task: Task
        :param agent_name: Agent name
        :type agent_name: str
        :return:
        """
        search = TaskKnowledgebase.generate_tuple(job_id=task.job_id, task_id=task.id, destination=task.destination, agent="none", status="open")
        new = TaskKnowledgebase.generate_tuple(job_id=task.job_id, task_id=task.id, destination=task.destination, agent=agent_name, status="assigned")

        ret_value = self.__kb_client.update(search, new, push_without_existing = True)
        return ret_value

    def finish_task(self, task, agent_name):
        """
        Marks a task as finished
        :param task: The task
        :type task: Task
        :param agent_name: Agent name
        :type agent_name: str
        :return:
        """
        search = TaskKnowledgebase.generate_tuple(job_id=task.job_id, task_id=task.id, destination=task.destination, agent=agent_name, status="assigned")
        new = TaskKnowledgebase.generate_tuple(job_id=task.job_id, task_id=task.id, destination=task.destination, agent="none", status="finished")

        ret_value = self._kb_client.update(search, new, push_without_existing = True)
        return ret_value

    def get_tasks(self, agent_name, status="*", job_id="*", destination="*", task_id="*"):
        """
        Returns the curretn tasks of an agent
        :param agent_name:
        :param status:
        :param job_id:
        :param destination:
        :param task_id:
        :return:
        """
        tuple = TaskKnowledgebase.generate_tuple(
            job_id=job_id,
            task_id=task_id,
            destination=destination,
            agent=agent_name,
            status=status)

        tasks = []

        for fact in  self._kb_client.all(tuple):
            tasks.append(TaskKnowledgebase.generate_task_from_fact(fact))
        return tasks
