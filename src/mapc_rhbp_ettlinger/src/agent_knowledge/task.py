#!/usr/bin/env python2

from mac_ros_bridge.msg import Position
from mapc_rhbp_ettlinger.msg import Task

from agent_knowledge.base_knowledge import BaseKnowledgeBase
from agent_knowledge.local_base_knowledge import BaseLocalKnowledgeBase
from common_utils import etti_logging
from common_utils.singleton import Singleton

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.knowledgebase.movement')


class TaskKnowledgeBase(BaseLocalKnowledgeBase):
    __metaclass__ = Singleton

    KNOWLEDGE_BASE_NAME = "knowledgeBaseNode"

    INDEX_MOVEMENT_AGENT_NAME = 1
    INDEX_MOVEMENT_TYPE = 2
    INDEX_MOVEMENT_ID = 3
    INDEX_MOVEMENT_LAT = 4
    INDEX_MOVEMENT_LONG = 5
    INDEX_MOVEMENT_TASK = 6

    TYPE_CHARGING_STATION = "charging_station"
    TYPE_EXPLORATION = "exploration"
    TYPE_GATHERING = "gathering"
    TYPE_ASSEMBLE = "assemble"
    TYPE_DELIVER = "deliver"
    TYPE_BUILD_WELL = "build_well"
    TYPE_HOARDING = "hoarding"

    PRIORITY_TASKS = [TYPE_BUILD_WELL, TYPE_DELIVER, TYPE_ASSEMBLE]

    def __init__(self):
        super(TaskKnowledgeBase, self).__init__(
            knowledge_base_name=TaskKnowledgeBase.KNOWLEDGE_BASE_NAME)

    @staticmethod
    def generate_tuple(agent_name="*", type="*", id="*", lat="*", long="*", task="*"):
        return 'task', agent_name, type, str(id), str(lat), str(long), str(task)

    @staticmethod
    def generate_task_from_fact(fact):
        """
        Generates a Movement object from a Knowledgebase fact
        :param fact: The fact
        :type fact: list
        :return: Movement
        """

        id_string = fact[TaskKnowledgeBase.INDEX_MOVEMENT_ID]
        if id_string is not '':
            id_int = int(id_string)
        else:
            id_int = 0
        task = Task(
            type=fact[TaskKnowledgeBase.INDEX_MOVEMENT_TYPE],
            agent_name=fact[TaskKnowledgeBase.INDEX_MOVEMENT_AGENT_NAME],
            pos=Position(
                lat=float(fact[TaskKnowledgeBase.INDEX_MOVEMENT_LAT]),
                long=float(fact[TaskKnowledgeBase.INDEX_MOVEMENT_LONG])
            ),
            task=fact[TaskKnowledgeBase.INDEX_MOVEMENT_TASK],
            id=id_int,
        )
        return task

    @staticmethod
    def generate_fact_from_task(task):
        """
        Generates a Knowledgebase fact from a Movement object
        :param task: The task object
        :type task: Task
        :return:
        """

        return TaskKnowledgeBase.generate_tuple(
            agent_name=task.agent_name,
            type=task.type,
            lat=task.pos.lat,
            long=task.pos.long,
            task=task.task,
            id=task.id if task.id is not 0 else '',
        )

    def create_task(self, task):

        search = TaskKnowledgeBase.generate_tuple(agent_name=task.agent_name, type=task.type)
        new = TaskKnowledgeBase.generate_fact_from_task(task)

        ettilog.loginfo("MovementKnowledge(%s:%s):: Moving to %s ", task.agent_name, task.type,
                        task.task)
        return self.kb_update(search, new, push_without_existing=True)

    def finish_task(self, agent_name, type, task="*"):
        """
        Stops the task of acertain identifier
        :param agent_name:
        :param identifier_name:
        :return:
        """
        search = TaskKnowledgeBase.generate_tuple(agent_name=agent_name, type=type, task=task)
        ettilog.loginfo("MovementKnowledge(%s:%s):: Stopping task", agent_name, type)

        return self.kb_pop(search)


    def get_task(self, agent_name, type):
        """
        Returns the current active task of a identifier
        :param agent_name:
        :param type:
        :return: Movement
        """

        search = TaskKnowledgeBase.generate_tuple(
            agent_name=agent_name,
            type=type)
        fact = self.kb_peek(search)



        if fact is not None:
            return TaskKnowledgeBase.generate_task_from_fact(fact)
        else:
            return None


    def get_tasks(self, type="*", task="*"):

        search = TaskKnowledgeBase.generate_tuple(
            task=task,
            type=type)
        facts = self.kb_fetch_all(search)

        res = []

        for fact in facts:
            res.append(TaskKnowledgeBase.generate_task_from_fact(fact))

        return res

    def has_priority_task(self, agent_name):
        search = TaskKnowledgeBase.generate_tuple(
            agent_name=agent_name)

        has_priority_task = False
        for fact in self.kb_fetch_all(search):
            if fact[TaskKnowledgeBase.INDEX_MOVEMENT_TYPE] in TaskKnowledgeBase.PRIORITY_TASKS:
                has_priority_task = True

        return has_priority_task

    def update_task(self, task, new_task):
        task_fact = self.generate_fact_from_task(task)
        new_task_fact = self.generate_fact_from_task(new_task)
        return self.kb_update(new_task_fact, task_fact, push_without_existing=True)

