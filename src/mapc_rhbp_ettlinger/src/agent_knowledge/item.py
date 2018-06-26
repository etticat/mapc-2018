#!/usr/bin/env python2

from knowledge_base.knowledge_base_manager import KnowledgeBase
from mapc_rhbp_ettlinger.msg import StockItem

from agent_knowledge.base_knowledge import BaseKnowledgeBase
from common_utils import etti_logging
from common_utils.singleton import Singleton

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.knowledgebase.item')


class StockItemBaseKnowledge(BaseKnowledgeBase):
    __metaclass__ = Singleton

    KNOWLEDGE_BASE_NAME = KnowledgeBase.DEFAULT_NAME

    INDEX_AGENT = 1
    INDEX_ITEM = 2
    INDEX_AMOUNT = 3
    INDEX_GOAL = 4

    def __init__(self):
        super(StockItemBaseKnowledge, self).__init__(
            knowledge_base_name=StockItemBaseKnowledge.KNOWLEDGE_BASE_NAME)

    @staticmethod
    def generate_tuple(agent_name="*", item="*", amount="*", goal="*"):
        return 'stock', agent_name, item, str(amount), str(goal)

    @staticmethod
    def generate_stock_item_from_fact(fact):
        """
        Generates a StockItem from a KnowledgeBase fact
        :param fact: The fact
        :type fact: list
        :return: StockItem
        """
        task = StockItem(
            agent=fact[StockItemBaseKnowledge.INDEX_AGENT],
            item=fact[StockItemBaseKnowledge.INDEX_ITEM],
            amount=int(fact[StockItemBaseKnowledge.INDEX_AMOUNT]),
            goal=int(fact[StockItemBaseKnowledge.INDEX_GOAL])
        )
        return task

    @staticmethod
    def generate_fact_from_stock_item(stock_item):
        """
        :param stock_item:
        :type stock_item: StockItem
        :return:
        """
        return StockItemBaseKnowledge.generate_tuple(
            agent_name=stock_item.agent,
            item=stock_item.item,
            amount=stock_item.amount,
            goal=stock_item.goal
        )

    def update_stock(self, stock_item):
        """
        Saves a new stock_item to the Knoledgebase
        :param stock_item:
        :type stock_item: StockItem
        :return:
        """
        search = StockItemBaseKnowledge.generate_tuple(
            agent_name=stock_item.agent,
            item=stock_item.item
        )
        new = StockItemBaseKnowledge.generate_fact_from_stock_item(stock_item)
        self._kb_client.update(search, new, push_without_existing=True)

    def get_total_stock(self):
        all_tuple = self.generate_tuple()
        res = {}

        facts = self._kb_client.all(all_tuple)

        for fact in facts:
            stock_item = StockItemBaseKnowledge.generate_stock_item_from_fact(fact)
            res[stock_item.item] = res.get(stock_item.item, 0) + max(stock_item.amount, stock_item.goal)
        return res

    def get_total_stock_and_goals(self):
        all_tuple = self.generate_tuple()
        res = {}

        facts = self._kb_client.all(all_tuple)

        if facts is not None:
            for fact in facts:
                stock_item = StockItemBaseKnowledge.generate_stock_item_from_fact(fact)

                if stock_item.item not in res.keys():
                    res[stock_item.item] = {
                        "stock": 0,
                        "goal": 0
                    }

                res[stock_item.item]["stock"] += stock_item.amount
                res[stock_item.item]["goal"] += max(stock_item.goal, stock_item.amount)
        else:
            ettilog.logerr("Error reading from db")
        return res
