import rospy
from knowledge_base.knowledge_base_client import KnowledgeBaseClient
from knowledge_base.knowledge_base_manager import KnowledgeBase


class BaseKnowledgebase(object):

    kb_instance=None

    def __init__(self):
        if BaseKnowledgebase.kb_instance == None:
            rospy.logerr("init kb")
            BaseKnowledgebase.kb_instance = KnowledgeBaseClient(knowledge_base_name = KnowledgeBase.DEFAULT_NAME)
        self._kb_client = BaseKnowledgebase.kb_instance