#!/usr/bin/env python2

import rospy
from mac_ros_bridge.msg import Position
from mapc_rhbp_ettlinger.msg import WellTask

from agent_knowledge.base_knowledge import BaseKnowledgebase

class WellTaskKnowledgebase(BaseKnowledgebase):

    INDEX_AGENT_NAME = 1
    INDEX_LAT = 2
    INDEX_LONG = 3
    INDEX_WELL_TYPE = 4
    INDEX_BUILT = 5

    @staticmethod
    def generate_tuple(agent_name="*", lat="*", long="*", well_type="*", built="*"):
        return 'well_task', agent_name, str(lat), str(long), well_type, str(built)

    @staticmethod
    def generate_well_task_from_fact(fact):
        """
        Generates a WellTask from a Knowledgebase fact
        :param fact: The fact
        :type fact: list
        :return: WellTask
        """
        if fact is None:
            return None

        well_task = WellTask(
            agent_name = fact[WellTaskKnowledgebase.INDEX_AGENT_NAME],
            pos = Position(
                lat=float(fact[WellTaskKnowledgebase.INDEX_LAT]),
                long=float(fact[WellTaskKnowledgebase.INDEX_LONG])),
            well_type=fact[WellTaskKnowledgebase.INDEX_WELL_TYPE],
            built=bool(WellTaskKnowledgebase.INDEX_BUILT)
        )
        return well_task

    @staticmethod
    def generate_fact_from_task(well_task):
        """
        Generates a knowledgebase fact rom a well_task
        :param well_task: The well_task
        :type well_task: WellTask
        :return: list
        """
        return WellTaskKnowledgebase.generate_tuple(
            agent_name=well_task.agent_name,
            lat=well_task.pos.lat,
            long=well_task.pos.long,
            well_type=well_task.well_type,
            built=well_task.built

        )

    def save_task(self, well_task):
        """
        Saves a new well_task to the Knoledgebase
        :param well_task:
        :type well_task: WellTask
        :return:
        """
        tuple = WellTaskKnowledgebase.generate_tuple(
            agent_name=well_task.agent_name
        )
        task_tuples = self._kb_client.all(tuple)

        if len(task_tuples) > 0:
            rospy.loginfo("TaskKnowledge:: Agent already has task assigned")
            return False
        else:
            new = WellTaskKnowledgebase.generate_fact_from_task(well_task)
            rospy.loginfo("TaskKnowledge:: WellTask %s saved", well_task.well_type)
            ret_value = self._kb_client.push(new)
            return True

    def end_well_task(self, agent_name):
        """
        Marks a well_task as finished
        :param well_task: The well_task
        :type well_task: WellTask
        :param agent_name: Agent name
        :type agent_name: str
        :return:
        """
        return self._kb_client.pop(self.generate_tuple(agent_name=agent_name))

    def build_finished(self, well_task):
        well_task.built = "*"
        search = self.generate_fact_from_task(well_task)
        well_task.built = True
        replace = self.generate_fact_from_task(well_task)
        return self._kb_client.update(search, replace, push_without_existing=False)

    def build_up_finished(self, well_task):
        well_task.built = "*"
        delete_fact = self.generate_fact_from_task(well_task)
        pop = self._kb_client.pop(delete_fact)

        return pop

    def get_task(self, agent_name="*", lat="*", long="*", well_type="*"):
        """
        Returns the curretn tasks of an agent
        :param agent_name:
        :param status:
        :param well_id:
        :param destination:
        :param task_id:
        :return:
        """
        tuple = WellTaskKnowledgebase.generate_tuple(
            agent_name=agent_name,
            lat=lat,
            long=long,
            well_type=well_type)

        return self.generate_well_task_from_fact(self._kb_client.peek(tuple))

    def get_tasks(self, tuple):

        res = []
        for fact in self._kb_client.all(tuple):
            res.append(self.generate_well_task_from_fact(fact))

        return res
