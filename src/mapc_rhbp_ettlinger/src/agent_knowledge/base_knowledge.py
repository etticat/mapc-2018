from knowledge_base.knowledge_base_client import KnowledgeBaseClient
from knowledge_base.knowledge_base_manager import KnowledgeBase

from common_utils import etti_logging

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.knowledge_base.base')


class BaseKnowledgeBase(object):

    KB_CLIENTS = {}

    def __init__(self, knowledge_base_name=KnowledgeBase.DEFAULT_NAME):
        self._kb_client = BaseKnowledgeBase.get_knowledgebase_object(knowledge_base_name=knowledge_base_name)

    @classmethod
    def get_knowledgebase_object(cls, knowledge_base_name):
        if BaseKnowledgeBase.KB_CLIENTS.get(knowledge_base_name, None) is None:
            BaseKnowledgeBase.KB_CLIENTS[knowledge_base_name] = KnowledgeBaseClient(knowledge_base_name=knowledge_base_name)
        return BaseKnowledgeBase.KB_CLIENTS[knowledge_base_name]



    def kb_update(self, new_task_fact, task_fact, push_without_existing):
        return self._kb_client.update(task_fact, new_task_fact, push_without_existing=push_without_existing)

    def kb_fetch_all(self, search):
        facts = self._kb_client.all(search)
        return facts

    def kb_peek(self, search):
        fact = self._kb_client.peek(search)
        return fact

    def kb_pop(self, search):
        return self._kb_client.pop(search)
