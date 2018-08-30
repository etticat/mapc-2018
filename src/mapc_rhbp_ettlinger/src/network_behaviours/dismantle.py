from behaviour_components.activators import GreedyActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition, Negation
from behaviour_components.goals import OfflineGoal
from behaviours.generic_action import GenericActionBehaviour
from behaviours.movement import GoToDestinationBehaviour
from decisions.dismantle_well import ExistingOpponentWellsDecision
from decisions.exploration_target import ExplorationDecision, ExploreCornersDecision
from network_behaviours.go_and_do import GoAndDoNetworkBehaviour
from provider.action_provider import Action
from provider.self_organisation_provider import SelfOrganisationProvider


class DismantleNetworkBehaviour(GoAndDoNetworkBehaviour):
    """
    Network behaviour that is responsible for exploring the environment
    """
    def __init__(self, agent_name, name, global_rhbp_components, **kwargs):

        self.self_organisation_provider = SelfOrganisationProvider(agent_name=agent_name)
        self._global_rhbp_components = global_rhbp_components
        super(DismantleNetworkBehaviour, self).__init__(mechanism=global_rhbp_components.opponent_wells_decision, name=name,
                                                        agent_name=agent_name,
                                                        global_rhbp_components=global_rhbp_components,
                                                        **kwargs)

        self.init_behaviours()
        self.init_goals()

    def init_behaviours(self):
        """
        Initialise exploration behaviour
        :return:
        """

        # When we reach a destination, just pick a new one and go there
        self._dismantle_behaviour = GenericActionBehaviour(
            agent_name=self._agent_name,
            plannerPrefix=self.get_manager_prefix(),
            name="dismantle_behaviour",
            action_type=Action.DISMANTLE
        )

        self.init_do_behaviour(self._dismantle_behaviour)

        self._dismantle_behaviour.add_effect(
            effect=Effect(
                sensor_name=self._global_rhbp_components.opponent_wells_sensor.name,
                sensor_type=bool,
                indicator=-1.0
            )
        )

    def init_goals(self):
        """
        Initialise the exploration goal
        :return:
        """
        self.goal = OfflineGoal(
            name='dismantle_goal',
            permanent=True,
            planner_prefix=self.get_manager_prefix(),
            conditions=[Negation(self._global_rhbp_components.opponent_well_exists_cond)])
        self.add_goal(self.goal)
