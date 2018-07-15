from behaviour_components.activators import GreedyActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition
from behaviour_components.goals import GoalBase
from behaviours.movement import GoToDestinationBehaviour
from network_behaviours.go_and_do import GoAndDoNetworkBehaviour
from provider.self_organisation_provider import SelfOrganisationProvider
from decisions.map_decisions import ExplorationDecision


class ExplorationNetworkBehaviour(GoAndDoNetworkBehaviour):
    """
    Network behaviour that is responsible for exploring the environment
    """
    def __init__(self, agent_name, name, global_rhbp_components, **kwargs):

        super(ExplorationNetworkBehaviour, self).__init__(mechanism=self.exploration_decision, name=name,
                                                          agent_name=agent_name,
                                                          global_rhbp_components=global_rhbp_components,
                                                          **kwargs)

        self.self_organisation_provider = SelfOrganisationProvider(agent_name=agent_name)
        self.exploration_decision = ExplorationDecision(self.self_organisation_provider._so_buffer)

        self.init_behaviours()
        self.init_goals()

    def init_behaviours(self):
        """
        Initialise exploration behaviour
        :return:
        """

        # When we reach a destination, just pick a new one and go there
        self._reset_destination_behaviour = GoToDestinationBehaviour(
            agent_name=self._agent_name,
            plannerPrefix=self.get_manager_prefix(),
            name=self.get_manager_prefix() + '_do_behaviour',
            mechanism=self.exploration_decision,
            recalculate_destination_every_step=True
        )

        self.init_do_behaviour(self._reset_destination_behaviour)

        self._reset_destination_behaviour.add_effect(
            effect=Effect(
                sensor_name=self._global_rhbp_components.resource_discovery_progress_sensor.name,
                sensor_type=float,
                indicator=1.0
            )
        )

    def init_goals(self):
        """
        Initialise the exploration goal
        :return:
        """
        self.goal = GoalBase(
            name='exploration_goal',
            permanent=True,
            plannerPrefix=self.get_manager_prefix(),
            conditions=[Condition(
                sensor=self._global_rhbp_components.resource_discovery_progress_sensor,
                activator=GreedyActivator()
            )])