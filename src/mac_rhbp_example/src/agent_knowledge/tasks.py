#!/usr/bin/env python2
import copy

from __builtin__ import xrange

import rospy
from knowledge_base.knowledge_base_client import KnowledgeBaseClient
from mac_rhbp_example.msg import Task
from mac_ros_bridge.msg import SimStart, Agent

from agent_common.agent_utils import AgentUtils


class TaskKnowledge():


    def __init__(self, agent_name):
        self.__kb_client = KnowledgeBaseClient(
            knowledge_base_name = "knowledgeBaseNode")

        self.products = {}

        rospy.Subscriber(
            AgentUtils.get_bridge_topic_prefix(agent_name=agent_name) + "start",
            SimStart,
            self._callback_sim_start)

        self._items_in_stock = {}
        self._assembled_items = {}

        self._sub_ref = rospy.Subscriber(AgentUtils.get_bridge_topic_agent(agent_name), Agent, self.subscription_callback_ref_topic)



    def subscription_callback_ref_topic(self, msg):
        """

        :param msg:
        :type msg: Agent
        :return:
        """
        self._items_in_stock  = {}

        for item in msg.items:
            self._items_in_stock[item.name] = self._items_in_stock.get(item.name, 0) + item.amount
    @staticmethod
    def get_tuple_task_creation(job_id="*", task_id="*", destination="*", agent="*", status="*", item="*"):
        return 'task', job_id, task_id, destination, agent, status, item


    INDEX_JOB_ID = 1
    INDEX_TASK_ID = 2
    INDEX_DESTINATION = 3
    INDEX_AGENT_NAME = 4
    INDEX_STATUS = 5
    INDEX_ITEM = 6


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
            status="*",
            item=task.item
        )
        task_tuples = self.__kb_client.all(tuple)

        if len(task_tuples) > 0:
            rospy.loginfo("TaskKnowledge:: Task %s%s already exists", task.job_id, task.id)
            return
        else:
            rospy.loginfo("TaskKnowledge:: Task %s%s saved", task.job_id, task.id)
            new = TaskKnowledge.get_tuple_task_creation(job_id=task.job_id, task_id=task.id, destination=task.destination, agent="none", status="none", item=task.item)
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

    def get_tasks(self, agent_name, status="*", job_id="*", destination="*", task_id="*"):
        """

        :param agent_name:
        :param status:
        :param job_id:
        :param destination:
        :param task_id:
        :return:
        """
        tuple = TaskKnowledge.get_tuple_task_creation(
            job_id=job_id,
            task_id=task_id,
            destination=destination,
            agent=agent_name,
            status=status)

        tasks = []

        for task_tuple in  self.__kb_client.all(tuple):

            task = Task(
                job_id=task_tuple[1],
                id=task_tuple[2],
                destination=task_tuple[3],
                agent=task_tuple[4],
                status=task_tuple[5],
                item=task_tuple[6])
            tasks.append(task)
        return tasks

    def get_required_finished_products(self, agent_name):
        all_required_finished_products = self.get_tasks(
            agent_name=agent_name,
            status="assigned")

        items_in_stock = copy.copy(self._items_in_stock)

        still_required_finished_products = []

        for product in all_required_finished_products:
            if (items_in_stock.get(product.item, 0) > 0): # we already have the item
                items_in_stock[product.item] -= 1
            else: # we still need the item
                still_required_finished_products.append(product.item)

        return still_required_finished_products

    def get_required_ingredients(self, agent_name):
        all_tasks = self.get_tasks(
            agent_name=agent_name,
            status="assigned")

        items_in_stock = copy.copy(self._items_in_stock)

        still_needed_ingredients = []

        while len(all_tasks ) > 0:
            task = all_tasks.pop()

            if (items_in_stock.get(task.item, 0) > 0):  # we already have the item
                items_in_stock[task.item] -= 1
            else:
                ingredients = list(self.products[task.item].consumed_items)
                while len(ingredients) > 0:
                    ingredient = ingredients.pop()

                    if len(self.products[ingredient.name].consumed_items) > 0: # If ingredient requires assembly
                        ingredients.extend(self.products[ingredient.name].consumed_items)
                    else:
                        for ingredient_index in xrange(ingredient.amount):
                            if items_in_stock.get(ingredient.name, 0) > 0:
                                items_in_stock[ingredient.name] -= 1
                            else:
                                still_needed_ingredients.append(ingredient.name)

        return still_needed_ingredients