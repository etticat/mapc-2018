#!/usr/bin/env python2

from mac_ros_bridge.msg import Position
from mapc_rhbp_ettlinger.msg import JobTask

from agent_knowledge.base_knowledge import BaseKnowledgebase
from common_utils import rhbp_logging

ettilog = rhbp_logging.LogManager(logger_name=rhbp_logging.LOGGER_DEFAULT_NAME + '.knowledgebase.tasks')

class JobKnowledgebase(BaseKnowledgebase):

    INDEX_AGENT_NAME = 1
    INDEX_JOB_ID = 2
    INDEX_LAT = 3
    INDEX_LONG = 4
    INDEX_ITEMS = 5

    @staticmethod
    def generate_tuple(agent_name="*", job_id="*", lat="*", long="*", items="*"):
        return 'job_task', agent_name, job_id, str(lat), str(long), items

    @staticmethod
    def generate_task_from_fact(fact):
        """
        Generates a JobTask from a Knowledgebase fact
        :param fact: The fact
        :type fact: list
        :return: JobTask
        """
        if fact is None:
            return None

        job_task = JobTask(
            agent_name = fact[JobKnowledgebase.INDEX_AGENT_NAME],
            job_id = fact[JobKnowledgebase.INDEX_JOB_ID],
            pos = Position(
                lat=float(fact[JobKnowledgebase.INDEX_LAT]),
                long=float(fact[JobKnowledgebase.INDEX_LONG])),
            items=fact[JobKnowledgebase.INDEX_ITEMS]
        )
        return job_task

    @staticmethod
    def generate_fact_from_task(job_task):
        """
        Generates a knowledgebase fact rom a job_task
        :param job_task: The job_task
        :type job_task: JobTask
        :return: list
        """
        return JobKnowledgebase.generate_tuple(
            job_id=job_task.job_id,
            agent_name=job_task.agent_name,
            lat=job_task.pos.lat,
            long=job_task.pos.long,
            items=job_task.items

        )

    def save_task(self, job_task):
        """
        Saves a new job_task to the Knoledgebase
        :param job_task:
        :type job_task: JobTask
        :return:
        """
        tuple = JobKnowledgebase.generate_tuple(
            agent_name=job_task.agent_name
        )
        task_tuples = self._kb_client.all(tuple)

        if len(task_tuples) > 0:
            ettilog.loginfo("TaskKnowledge:: Agent already has task assigned")
            return False
        else:
            new = JobKnowledgebase.generate_fact_from_task(job_task)
            ettilog.loginfo("TaskKnowledge:: JobTask %s saved", job_task.job_id)
            ret_value = self._kb_client.push(new)
            return True

    def end_job_task(self, job_id, agent_name):
        """
        Marks a job_task as finished
        :param job_task: The job_task
        :type job_task: JobTask
        :param agent_name: Agent name
        :type agent_name: str
        :return:
        """
        return self._kb_client.pop(self.generate_tuple(agent_name=agent_name, job_id=job_id))

    def get_task(self, agent_name="*", job_id="*", lat="*", long="*", items="*"):
        """
        Returns the curretn tasks of an agent
        :param agent_name:
        :param status:
        :param job_id:
        :param destination:
        :param task_id:
        :return:
        """
        tuple = JobKnowledgebase.generate_tuple(
            agent_name=agent_name,
            job_id=job_id,
            lat=lat,
            long=long,
            items=items)

        return self.generate_task_from_fact(self._kb_client.peek(tuple))
