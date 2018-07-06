from behaviours.assemble import AssembleProductBehaviour
from decisions.p_task_decision import CurrentTaskDecision
from network_behaviours.go_and_do import GoAndDoNetworkBehaviour


class AssembleNetworkBehaviour(GoAndDoNetworkBehaviour):

    def __init__(self, agent_name, name, sensor_map, **kwargs):

        super(AssembleNetworkBehaviour, self).__init__(
            agent_name=agent_name,
            sensor_map=sensor_map,
            name=name,
            mechanism=sensor_map.assemble_task_mechanism,
            **kwargs)

        assemble_behaviour = AssembleProductBehaviour(
            name=self.get_manager_prefix() + "_assemble_product_behaviour",
            agent_name=agent_name,
            mechanism=sensor_map.assemble_task_mechanism,
            plannerPrefix=self.get_manager_prefix()
        )
        self.init_do_behaviour(assemble_behaviour)