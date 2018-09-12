from behaviour_components.condition_elements import Effect
from behaviour_components.goals import GoalBase
from behaviours.generic_action import GenericActionBehaviour
from network_behaviours.go_and_do import GoAndDoNetworkBehaviour
from provider.action_provider import Action


class GatheringNetworkBehaviour(GoAndDoNetworkBehaviour):
    """
    Network behaviour responsible for gathering ingredients
    """

    def __init__(self, agent_name, name, shared_components, **kwargs):
        super(GatheringNetworkBehaviour, self).__init__(
            agent_name=agent_name,
            shared_components=shared_components,
            name=name,
            mechanism=shared_components.gather_decision_decision,
            **kwargs)

        self._agent_name = self._agent_name

        self.gather_behaviour = GenericActionBehaviour(
            name="gather_behaviour",
            action_type=Action.GATHER,
            agent_name=self._agent_name,
            plannerPrefix=self.get_manager_prefix()
        )

        self.gather_behaviour.add_effect(
            effect=Effect(
                sensor_name=self._shared_components.load_factor_sensor.name,
                indicator=1.0,
                sensor_type=float

            )
        )

        self.init_do_behaviour(self.gather_behaviour)

        self.gather_goal = GoalBase(
            name='gather_goal',
            permanent=True,
            priority=50,
            planner_prefix=self._agent_name,
            conditions=[self._shared_components.load_factor_condition])

    def stop(self):
        """
        When stopping to gather, remove goals
        :return:
        """
        super(GatheringNetworkBehaviour, self).stop()
        # Deleting task and cleaning up goals
        self._shared_components.gather_decision_decision.end_gathering()

    def do_step(self):
        """
        Pick best gather option each step
        :return:
        """
        self._shared_components.gather_decision_decision.calc_value()
        super(GatheringNetworkBehaviour, self).do_step()
