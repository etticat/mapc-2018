from agent_knowledge.task import TaskBaseKnowledge
from behaviour_components.behaviours import BehaviourBase


class FinishTaskBehaviour(BehaviourBase):

    def __init__(self, agent_name, type, **kwargs):
        super(FinishTaskBehaviour, self) \
            .__init__(
            requires_execution_steps=True,
            **kwargs)
        self._type = type
        self._agent_name = agent_name
        self._task_knowledge_base = TaskBaseKnowledge()

    def do_step(self):
        self._task_knowledge_base.finish_task(agent_name=self._agent_name, type=self._type)
        super(FinishTaskBehaviour, self).do_step()
