#!/usr/bin/env python2

import rospy
from mapc_rhbp_ettlinger.msg import Task, StockItem

from agent_knowledge.base_knowledge import BaseKnowledgebase

class StockItemKnowledgebase(BaseKnowledgebase):

    INDEX_AGENT = 1
    INDEX_ITEM = 2
    INDEX_AMOUNT = 3
    INDEX_GOAL = 4


    @staticmethod
    def generate_tuple(agent_name="*", item="*", amount="*",goal="*"):
        return 'stock', agent_name, item, str(amount),str(goal)

    @staticmethod
    def generate_stock_item_from_fact(fact):
        """
        Generates a StockItem from a Knowledgebase fact
        :param fact: The fact
        :type fact: list
        :return: StockItem
        """
        task = StockItem(
            agent=fact[StockItemKnowledgebase.INDEX_AGENT],
            item=fact[StockItemKnowledgebase.INDEX_ITEM],
            amount=int(fact[StockItemKnowledgebase.INDEX_AMOUNT]),
            goal=int(fact[StockItemKnowledgebase.INDEX_GOAL])
        )
        return task

    @staticmethod
    def generate_fact_from_stock_item(stock_item):
        """
        Generates a knowledgebase fact rom a StockItem
        :param task: The hoarded item
        :type task: StockItem
        :return: list
        """
        return StockItemKnowledgebase.generate_tuple(
            agent_name=stock_item.agent,
            item=stock_item.item,
            amount=stock_item.amount,
            goal=stock_item.goal
        )

    def update_stock(self, stockItem):
        """
        Saves a new stock_item to the Knoledgebase
        :param stockItem:
        :type stockItem: StockItem
        :return:
        """
        search = StockItemKnowledgebase.generate_tuple(
            agent_name=stockItem.agent,
            item=stockItem.item
        )
        new = StockItemKnowledgebase.generate_fact_from_stock_item(stockItem)
        self._kb_client.update(search, new, push_without_existing = True)

    def get_total_stock(self):
        all = self.generate_tuple()
        res = {}

        facts = self._kb_client.all(all)

        for fact in facts:
            stockItem = StockItemKnowledgebase.generate_stock_item_from_fact(fact)
            res[stockItem.item] = res.get(stockItem.item,0) + max(stockItem.amount, stockItem.goal)
        return res

    def get_total_stock_and_goals(self):
        all = self.generate_tuple()
        res = {}

        facts = self._kb_client.all(all)

        if facts != None:
            for fact in facts:
                stockItem = StockItemKnowledgebase.generate_stock_item_from_fact(fact)

                if stockItem.item not in res.keys():
                    res[stockItem.item] = {
                        "stock": 0,
                        "goal": 0
                    }



                res[stockItem.item]["stock"] += stockItem.amount
                res[stockItem.item]["goal"] += stockItem.amount
        else:
            rospy.logerr("Error reading from db")
        return res