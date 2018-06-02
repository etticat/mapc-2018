from knowledge_base.knowledge_base_client import KnowledgeBaseClient
from knowledge_base.knowledge_base_manager import KnowledgeBase


class BaseKnowledgebase(object):


    def __init__(self):
        self._kb_client = KnowledgeBaseClient(
            knowledge_base_name = KnowledgeBase.DEFAULT_NAME)