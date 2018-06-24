from knowledge_base.knowledge_base_client import KnowledgeBaseClient
from knowledge_base.knowledge_base_manager import KnowledgeBase

from common_utils import etti_logging

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.knowledge_base.base')


class BaseKnowledgeBase(object):

    def __init__(self, knowledge_base_name=KnowledgeBase.DEFAULT_NAME):
        self._kb_client = BaseKnowledgeBase.kb_instance = KnowledgeBaseClient(knowledge_base_name=knowledge_base_name)
