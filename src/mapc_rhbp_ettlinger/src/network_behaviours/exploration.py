from behaviour_components.activators import GreedyActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition
from behaviour_components.goals import GoalBase
from behaviours.movement import GoToDestinationBehaviour
from network_behaviours.go_and_do import GoAndDoNetworkBehaviour
from provider.self_organisation_provider import SelfOrganisationProvider
from self_organisation.decisions import ExplorationDecision


class ExplorationNetworkBehaviour(GoAndDoNetworkBehaviour):
    def __init__(self, agent_name, name, sensor_map, **kwargs):


        self.self_organisation_provider = SelfOrganisationProvider(agent_name=agent_name)

        self.exploration_decision = ExplorationDecision(self.self_organisation_provider.buffer)

        super(ExplorationNetworkBehaviour, self).__init__(mechanism=self.exploration_decision, name=name, agent_name=agent_name, sensor_map=sensor_map,
                                                          **kwargs)
        self.init_destination_step_sensor()


        self._reset_destination_behaviour = GoToDestinationBehaviour(
            agent_name=self._agent_name,
            plannerPrefix=self.get_manager_prefix(),
            name=self.get_manager_prefix()  + '_do_behaviour',
            mechanism=self.exploration_decision,
            recalculate_destination_every_step=True
        )

        self.init_do_behaviour(self._reset_destination_behaviour)

        self.goal = GoalBase(
            name='job_fulfillment_goal',
            permanent=True,
            plannerPrefix=self.get_manager_prefix(),
            conditions=[Condition(sensor=self._sensor_map.resource_discovery_progress_sensor,
                        activator=GreedyActivator())])

        self._reset_destination_behaviour.add_effect(
            effect=Effect(
                sensor_name=self._sensor_map.resource_discovery_progress_sensor.name,
                sensor_type=float,
                indicator=1.0
            )
        )