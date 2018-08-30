from behaviour_components.activators import GreedyActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition
from behaviour_components.goals import OfflineGoal
from behaviours.movement import GoToDestinationBehaviour
from network_behaviours.go_and_do import GoAndDoNetworkBehaviour


class ExplorationNetworkBehaviour(GoAndDoNetworkBehaviour):
    """
    Network behaviour that is responsible for exploring the environment
    """
    def __init__(self, agent_name, name, global_rhbp_components, exploration_mechanism, **kwargs):

        self.exploration_mechanism = exploration_mechanism
        super(ExplorationNetworkBehaviour, self).__init__(mechanism=self.exploration_mechanism, name=name,
                                                          agent_name=agent_name,
                                                          min_charge=5,
                                                          use_in_facility_flag=False,
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
        self._reset_destination_behaviour = GoToDestinationBehaviour(
            agent_name=self._agent_name,
            plannerPrefix=self.get_manager_prefix(),
            name=self.get_manager_prefix() + '_do_behaviour',
            mechanism=self.exploration_mechanism,
            recalculate_destination_every_step=True
        )

        self.init_do_behaviour(self._reset_destination_behaviour)
        self.apply_charging_restrictions(self._reset_destination_behaviour)

        self._reset_destination_behaviour.add_effect(
            effect=Effect(
                sensor_name=self._global_rhbp_components.resource_discovery_progress_sensor.name,
                sensor_type=float,
                indicator=1.0
            )
        )
        self._go_behaviour.add_effect(
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
        self.goal = OfflineGoal(
            name=self.get_manager_prefix() + 'exploration_goal',
            permanent=True,
            planner_prefix=self.get_manager_prefix(),
            conditions=[Condition(
                sensor=self._global_rhbp_components.resource_discovery_progress_sensor,
                activator=GreedyActivator()
            )])
        self.add_goal(self.goal)
