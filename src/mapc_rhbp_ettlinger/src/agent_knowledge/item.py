#!/usr/bin/env python2

from mapc_rhbp_ettlinger.msg import StockItem

from agent_knowledge.base_knowledge import BaseKnowledgeBase
from common_utils import etti_logging
from common_utils.singleton import Singleton

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.knowledgebase.item')


class StockItemKnowledgeBase(BaseKnowledgeBase):
    __metaclass__ = Singleton

    KNOWLEDGE_BASE_NAME = "knowledgeBaseNode"

    INDEX_AGENT = 1
    INDEX_AMOUNTS = 2
    INDEX_GOALS = 3

    def __init__(self):
        super(StockItemKnowledgeBase, self).__init__(
            knowledge_base_name=StockItemKnowledgeBase.KNOWLEDGE_BASE_NAME)

    @staticmethod
    def generate_tuple(agent_name="*", amounts="*", goals="*"):

        amounts = amounts if amounts == "*" else StockItemKnowledgeBase.item_dict_to_string(amounts)
        goals = goals if goals == "*" else StockItemKnowledgeBase.item_dict_to_string(goals)
        return 'stock', agent_name, amounts, goals

    @staticmethod
    def generate_stock_item_from_fact(fact):
        """
        Generates a StockItem from a KnowledgeBase fact
        :param fact: The fact
        :type fact: list
        :return: StockItem
        """
        task = StockItem(
            agent=fact[StockItemKnowledgeBase.INDEX_AGENT],
            amounts=StockItemKnowledgeBase.string_to_item_dict(fact[StockItemKnowledgeBase.INDEX_AMOUNTS]),
            goals=StockItemKnowledgeBase.string_to_item_dict(fact[StockItemKnowledgeBase.INDEX_GOALS]),
        )
        return task

    @staticmethod
    def string_to_item_dict(string):
        res = {}
        if string == "":
            return res
        for item_with_value in string.split(","):
            key, value = item_with_value.split(":")
            res[key] = int(value)

        return res

    @staticmethod
    def item_dict_to_string(item_dict):
        return ",".join([key + ":" + str(value) for key, value in item_dict.iteritems()])


    @staticmethod
    def generate_fact_from_stock_item(stock_item):
        """
        :param stock_item:
        :type stock_item: StockItem
        :return:
        """
        return StockItemKnowledgeBase.generate_tuple(
            agent_name=stock_item.agent,
            amounts=stock_item.amounts,
            goals=stock_item.goals
        )

    def update_stock(self, stock_item):
        """
        Saves a new stock_item to the Knoledgebase
        :param stock_item:
        :type stock_item: StockItem
        :return:
        """
        search = StockItemKnowledgeBase.generate_tuple(
            agent_name=stock_item.agent
        )
        new = StockItemKnowledgeBase.generate_fact_from_stock_item(stock_item)
        self._kb_client.update(search, new, push_without_existing=True)

    def get_total_stock(self):
        all_tuple = self.generate_tuple()
        res = {}

        facts = self._kb_client.all(all_tuple)

        for fact in facts:
            stock_item = StockItemKnowledgeBase.generate_stock_item_from_fact(fact)
            for item in set(stock_item.goals) | set(stock_item.amounts):
                res[item] = res.get(item, 0) + max(stock_item.amounts[item], stock_item.goals[item])
        return res

    def get_total_stock_and_goals(self):
        all_tuple = self.generate_tuple()
        res = StockItem(agent="all", goals={}, amounts={})

        facts = self._kb_client.all(all_tuple)

        if facts is not None:
            for fact in facts:
                stock_item = StockItemKnowledgeBase.generate_stock_item_from_fact(fact)

                for item in set(stock_item.goals) | set(stock_item.amounts):
                    res.amounts[item] = res.amounts.get(item, 0) + stock_item.amounts.get(item,0)
                    res.goals[item] = res.goals.get(item, 0) + max(stock_item.goals.get(item,0), stock_item.amounts.get(item,0))
        else:
            ettilog.logerr("Error reading from db")
        return res
