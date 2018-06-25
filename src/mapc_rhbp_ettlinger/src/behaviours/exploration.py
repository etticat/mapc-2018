from mapc_rhbp_ettlinger.msg import Task

from agent_knowledge.task import TaskKnowledgeBase
from behaviour_components.behaviours import BehaviourBase
from provider.simulation_provider import SimulationProvider


class ChooseDestinationBehaviour(BehaviourBase):
    def __init__(self, name, agent_name, type, **kwargs):
        super(ChooseDestinationBehaviour, self).__init__(name, requires_execution_steps=True, **kwargs)
        self.type = type
        self._simulation_provider = SimulationProvider()
        self._movement_knowledge_base = TaskKnowledgeBase()

        self.agent_name = agent_name

    def do_step(self):
        destination = self._simulation_provider.get_random_position()
        self._movement_knowledge_base.create_task(Task(
            type=self.type,
            agent_name=self.agent_name,
            pos=destination
        ))
