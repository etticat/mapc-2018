from agent_knowledge.task import TaskKnowledgeBase
from behaviour_components.sensors import Sensor
from common_utils import etti_logging

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.knowledge_base.base')



class LocalKnowledgeSensor(Sensor):

    def __init__(self, pattern, knowledge_base_name, optional=False, name=None):
        self.pattern = pattern
        super(LocalKnowledgeSensor, self).__init__(name=name, optional=optional, initial_value=False)
        self.task_knowledge_base = TaskKnowledgeBase()
        self.task_knowledge_base.register_update_listener(self._update_val)

    def _update_val(self):
        current_val = len(self.task_knowledge_base.kb_fetch_all(self.pattern)) > 0
        self.update(current_val)



class LocalKnowledgeFactSensor(Sensor):
    """
    Sensor, which provides information about a searched fact; returns list of
    all matching facts
    """

    def __init__(self, pattern, knowledge_base_name, optional=False, name=None,
                 initial_value=None):
        self.pattern = pattern
        super(LocalKnowledgeFactSensor, self).__init__(name=name, optional=optional, initial_value=initial_value)
        self.task_knowledge_base = TaskKnowledgeBase()
        self.task_knowledge_base.register_update_listener(self._update_val)

    def _update_val(self):
        current_val = self._reduce_facts(self.task_knowledge_base.kb_fetch_all(self.pattern))
        self.update(current_val)

    def _reduce_facts(self, param):
        return param


class LocalKnowledgeFirstFactSensor(LocalKnowledgeFactSensor):
    """
    Sensor, which provides the last value of the first found fact tuple, e.g. fact matches (a,b,c), it would return c
    """

    def __init__(self, pattern, knowledge_base_name, optional=False,
                 name=None, initial_value=None, index=-1):
        super(LocalKnowledgeFirstFactSensor, self).__init__(name=name, optional=optional, initial_value=initial_value,
                                                       pattern=pattern, knowledge_base_name=knowledge_base_name)
        self.index = index

    def _reduce_facts(self, facts):
        value = self._initial_value
        if len(facts) > 0:
            fact_tuple = facts.pop()  # only getting the first fact

            value = fact_tuple[self.index]
        return value