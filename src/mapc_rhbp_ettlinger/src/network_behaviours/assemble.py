from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Negation
from behaviour_components.goals import GoalBase
from behaviours.assemble import AssembleProductBehaviour
from network_behaviours.go_and_do import GoAndDoNetworkBehaviour


class AssembleNetworkBehaviour(GoAndDoNetworkBehaviour):
    """
    Network Behaviour responisble for performing assigned assemble tasks
    """

    def __init__(self, agent_name, name, global_rhbp_components, **kwargs):

        super(AssembleNetworkBehaviour, self).__init__(
            agent_name=agent_name,
            global_rhbp_components=global_rhbp_components,
            name=name,
            mechanism=global_rhbp_components.assemble_task_mechanism,
            **kwargs)

        assemble_behaviour = AssembleProductBehaviour(
            name=self.get_manager_prefix() + "_assemble_product_behaviour",
            agent_name=agent_name,
            mechanism=global_rhbp_components.assemble_task_mechanism,
            plannerPrefix=self.get_manager_prefix()
        )

        assemble_behaviour.add_effect(
            effect=Effect(
                sensor_name=global_rhbp_components.has_assemble_task_sensor.name,
                sensor_type=bool,
                indicator=-1.0
            )
        )

        self.assemble_goal = GoalBase(
            name='assemble_goal',
            permanent=True,
            priority=100,
            planner_prefix=self.get_manager_prefix(),
            conditions=[Negation(self._global_rhbp_components.has_assemble_task_assigned_cond)])

        self.init_do_behaviour(assemble_behaviour)