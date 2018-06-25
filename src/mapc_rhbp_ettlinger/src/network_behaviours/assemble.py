from agent_knowledge.task import TaskKnowledgeBase
from behaviours.assemble import AssembleProductBehaviour
from network_behaviours.go_and_do import GoAndDoNetworkBehaviour


class AssembleNetworkBehaviour(GoAndDoNetworkBehaviour):

    def __init__(self, agent_name, name, sensor_map, **kwargs):
        super(AssembleNetworkBehaviour, self).__init__(
            agent_name=agent_name,
            sensor_map=sensor_map,
            name=name,
            task_type=TaskKnowledgeBase.TYPE_ASSEMBLE,
            **kwargs)

        assemble_behaviour = AssembleProductBehaviour(
            name=self.get_manager_prefix() + "_assemble_product_behaviour",
            agent_name=agent_name,
            plannerPrefix=self.get_manager_prefix()
        )
        self.init_do_behaviour(assemble_behaviour)