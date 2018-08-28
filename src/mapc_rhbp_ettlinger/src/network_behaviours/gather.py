
from behaviour_components.activators import ThresholdActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition, Negation
from behaviour_components.goals import OfflineGoal
from behaviours.generic_action import GenericActionBehaviour
from behaviours.movement import GoToDestinationBehaviour
from network_behaviours.battery import BatteryChargingNetworkBehaviour
from network_behaviours.go_and_do import GoAndDoNetworkBehaviour
from provider.action_provider import Action
from provider.product_provider import ProductProvider
from rhbp_selforga.gradientsensor import GradientSensor, SENSOR
from sensor.general import SubtractionSensor
from sensor.movement import StepDistanceSensor


class GatheringNetworkBehaviour(GoAndDoNetworkBehaviour):
    """
    Network behaviour responsible for gathering ingredients
    """

    def __init__(self, agent_name, name, global_rhbp_components, **kwargs):

        super(GatheringNetworkBehaviour, self).__init__(
            agent_name=agent_name,
            global_rhbp_components=global_rhbp_components,
            name=name,
            mechanism=global_rhbp_components.gather_decision_mechanism,
            **kwargs)

        self._agent_name = self._agent_name

        self.gather_behviour = GenericActionBehaviour(
            name="gather_behaviour",
            action_type=Action.GATHER,
            agent_name=self._agent_name,
            plannerPrefix=self.get_manager_prefix()
        )

        self.gather_behviour.add_effect(
            effect=Effect(
                sensor_name=self._global_rhbp_components.load_factor_sensor.name,
                indicator=1.0,
                sensor_type=float

            )
        )

        self.init_do_behaviour(self.gather_behviour)

        self.gather_goal = OfflineGoal(
            name='gather_goal',
            permanent=True,
            priority=50,
            planner_prefix=self._agent_name,
            conditions=[self._global_rhbp_components.load_fullness_condition])

    def stop(self):
        """
        When stoping to gather, remove goals
        :return:
        """
        super(GatheringNetworkBehaviour, self).stop()
        # Deleting task and cleaning up goals
        self._global_rhbp_components.gather_decision_mechanism.end_gathering()