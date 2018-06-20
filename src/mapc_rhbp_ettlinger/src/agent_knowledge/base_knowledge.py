from knowledge_base.knowledge_base_client import KnowledgeBaseClient
from knowledge_base.knowledge_base_manager import KnowledgeBase

from common_utils import rhbp_logging

ettilog = rhbp_logging.LogManager(logger_name=rhbp_logging.LOGGER_DEFAULT_NAME + '.knowledgebase.base')

class BaseKnowledgebase(object):


    def __init__(self):
        self._kb_client = BaseKnowledgebase.kb_instance = KnowledgeBaseClient(knowledge_base_name = KnowledgeBase.DEFAULT_NAME)