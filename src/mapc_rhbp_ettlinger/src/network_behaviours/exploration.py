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