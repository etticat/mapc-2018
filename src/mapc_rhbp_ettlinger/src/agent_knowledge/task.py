#!/usr/bin/env python2

from knowledge_base.knowledge_base_manager import KnowledgeBase
from mac_ros_bridge.msg import Position
from mapc_rhbp_ettlinger.msg import Task

from agent_knowledge.base_knowledge import BaseKnowledgebase
from common_utils import rhbp_logging

ettilog = rhbp_logging.LogManager(logger_name=rhbp_logging.LOGGER_DEFAULT_NAME + '.knowledgebase.movement')


class TaskKnowledgebase(BaseKnowledgebase):

    KNOWLEDGE_BASE_NAME = KnowledgeBase.DEFAULT_NAME

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

    def __init__(self):
        super(TaskKnowledgebase, self).__init__(
            knowledge_base_name=TaskKnowledgebase.KNOWLEDGE_BASE_NAME)

    @staticmethod
    def generate_tuple(agent_name="*", type="*", id="*", lat="*", long="*", task="*"):
        return 'moving', agent_name, type, str(id), str(lat), str(long), str(task)

    @staticmethod
    def generate_task_from_fact(fact):
        """
        Generates a Movement object from a Knowledgebase fact
        :param fact: The fact
        :type fact: list
        :return: Movement
        """

        movement = Task(
            type=fact[TaskKnowledgebase.INDEX_MOVEMENT_TYPE],
            agent_name=fact[TaskKnowledgebase.INDEX_MOVEMENT_AGENT_NAME],
            pos=Position(
                lat=float(fact[TaskKnowledgebase.INDEX_MOVEMENT_LAT]),
                long=float(fact[TaskKnowledgebase.INDEX_MOVEMENT_LONG])
            ),
            task=fact[TaskKnowledgebase.INDEX_MOVEMENT_TASK],
            id=fact[TaskKnowledgebase.INDEX_MOVEMENT_ID],
        )
        return movement

    @staticmethod
    def generate_fact_from_movement(movement):
        """
        Generates a Knowledgebase fact from a Movement object
        :param movement: The movement object
        :type movement: Task
        :return:
        """

        return TaskKnowledgebase.generate_tuple(
            agent_name=movement.agent_name,
            type=movement.type,
            lat=movement.pos.lat,
            long=movement.pos.long,
            task=movement.task,
            id=movement.id,
        )

    def create_task(self, movement):
        """
        Starts movement to a new destination.
        :param agent_name:
        :param identifier_name:
        :param destinationPos:
        :param destination:
        :return:
        """
        search = TaskKnowledgebase.generate_tuple(agent_name=movement.agent_name, type=movement.type)
        new = TaskKnowledgebase.generate_fact_from_movement(movement)

        ettilog.loginfo("MovementKnowledge(%s:%s):: Moving to %s ", movement.agent_name, movement.type,
                        movement.task)
        ret_value = self._kb_client.update(search, new, push_without_existing=True)

    def finish_task(self, agent_name, type):
        """
        Stops the movement of acertain identifier
        :param agent_name:
        :param identifier_name:
        :return:
        """
        search = TaskKnowledgebase.generate_tuple(agent_name=agent_name, type=type)
        ettilog.loginfo("MovementKnowledge(%s:%s):: Stopping movement", agent_name, type)

        return self._kb_client.pop(search)

    def get_task(self, agent_name="*", type="*", task="*"):
        """
        Returns the current active movement of a identifier
        :param agent_name:
        :param type:
        :return: Movement
        """

        search = TaskKnowledgebase.generate_tuple(
            agent_name=agent_name,
            task=task,
            type=type)
        fact = self._kb_client.peek(search)



        if fact is not None:
            return TaskKnowledgebase.generate_task_from_fact(fact)
        else:
            return None

    def get_tasks(self, type="*", task="*"):

        search = TaskKnowledgebase.generate_tuple(
            task=task,
            type=type)
        facts = self._kb_client.all(search)

        res = []

        for fact in facts:
            res.append(TaskKnowledgebase.generate_task_from_fact(fact))

        return res
