import rospy
from knowledge_base.knowledge_base_client import KnowledgeBaseClient
from knowledge_base.knowledge_base_manager import KnowledgeBase

from common_utils import etti_logging

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.knowledge_base.base')


class BaseLocalKnowledgeBase(object):

    KB_CLIENTS = {}

    def __init__(self, knowledge_base_name=KnowledgeBase.DEFAULT_NAME):
        self.listeners = []
        self._kb_client = BaseLocalKnowledgeBase.get_knowledgebase_object(knowledge_base_name=knowledge_base_name)
        
        self.tuples = []

    @classmethod
    def get_knowledgebase_object(cls, knowledge_base_name):
        if BaseLocalKnowledgeBase.KB_CLIENTS.get(knowledge_base_name, None) is None:
            BaseLocalKnowledgeBase.KB_CLIENTS[knowledge_base_name] = KnowledgeBaseClient(knowledge_base_name=knowledge_base_name)
        return BaseLocalKnowledgeBase.KB_CLIENTS[knowledge_base_name]
    
    
    

    def kb_update(self, search, replace, push_without_existing):
        existing = self.kb_pop(search, notify_listeners=False)
        if len(existing) > 0 or push_without_existing:
            self.tuples.append(replace)
            self.notify_listeners()
            return True
        else:
            return False

    def kb_fetch_all(self, search):
        res = []
        
        for tuple in self.tuples:
            tuple_matches = True
            for i in range(len(tuple)):
                if tuple[i] == search[i] or search[i] == "*":
                    continue
                else: 
                    tuple_matches = False
                    break
            if tuple_matches:
                res.append(tuple)
        
        return res

    def kb_peek(self, search):
        all = self.kb_fetch_all(search)
        if len(all) > 0:
            return all[0]
        else:
            return None

    def kb_pop(self, search, notify_listeners=True):
        facts = self.kb_fetch_all(search)
        for fact in facts:
            self.tuples.remove(fact)

        if notify_listeners:
            self.notify_listeners()
        return facts

    def register_update_listener(self, _update_val):
        self.listeners.append(_update_val)

    def notify_listeners(self):
        for listener in self.listeners:
            listener()