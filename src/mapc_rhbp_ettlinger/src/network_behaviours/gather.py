
from behaviour_components.activators import ThresholdActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition, Negation
from behaviour_components.goals import OfflineGoal
from behaviours.generic_action import GenericActionBehaviour
from behaviours.movement import GoToDestinationBehaviour
from network_behaviours.battery import BatteryChargingNetworkBehaviour
from provider.action_provider import Action
from provider.product_provider import ProductProvider
from rhbp_selforga.gradientsensor import GradientSensor, SENSOR
from sensor.general import SubtractionSensor
from sensor.movement import StepDistanceSensor


class GatheringNetworkBehaviour(BatteryChargingNetworkBehaviour):
    """
    Network behaviour responsible for gathering ingredients
    TODO: This could also inherit from GoAndDoBehaviour to safe a little bit of code
    """

    def __init__(self, agent_name, name, global_rhbp_components, **kwargs):
        super(GatheringNetworkBehaviour, self).__init__(name=name, agent_name=agent_name, global_rhbp_components=global_rhbp_components,
                                                         **kwargs)

        self._agent_name = self._agent_name

        self._product_provider = ProductProvider(agent_name=self._agent_name)

        self.init_sensors(global_rhbp_components=global_rhbp_components)
        self.init_go_to_resource_behaviour(global_rhbp_components)
        self.init_gather_behaviour()

        self.apply_charging_restrictions(self.go_to_resource_node_behaviour)

        self.charge_goal = OfflineGoal(
            name='gather_goal',
            permanent=True,
            priority=100,
            planner_prefix=self.get_manager_prefix(),
            conditions=[self._global_rhbp_components.load_fullness_condition])

    def init_gather_behaviour(self):
        """
        Initialise gather behaviour
        :return:
        """
        ############### Gathering ##########################
        self.gather_behviour = GenericActionBehaviour(
            name="gather_behaviour",
            action_type=Action.GATHER,
            agent_name=self._agent_name,
            plannerPrefix=self.get_manager_prefix()
        )
        # Only gather if we are at the intended resource node
        self.gather_behviour.add_precondition(
            precondition=self.at_resource_node_condition)

        self.gather_behviour.add_precondition(
            precondition=self.next_item_fits_in_storage_cond)

        self.gather_behviour.add_effect(
            effect=Effect(
                sensor_name=self._global_rhbp_components.load_factor_sensor.name,
                indicator=1.0,
                sensor_type=float

            )
        )

    def init_go_to_resource_behaviour(self, global_rhbp_components):
        """
        Initialise movement behaviour
        :param global_rhbp_components:
        :return:
        """
        ################ Going to resource #########################
        self.go_to_resource_node_behaviour = GoToDestinationBehaviour(
            agent_name=self._agent_name,
            name="go_to_resource_node_behaviour",
            plannerPrefix=self.get_manager_prefix(),
            mechanism=global_rhbp_components.gather_decision_mechanism
        )

        self.gather_target_sensor = GradientSensor(
            mechanism=self._global_rhbp_components.gather_decision_mechanism,
            name="gather_target_sensor",
            sensor_type=SENSOR.VALUE
        )
        # Sensor to check distance to charging station
        self.target_step_sensor = StepDistanceSensor(
            name='gather_target_step_sensor',
            agent_name=self._agent_name,
            position_sensor_2=self.gather_target_sensor,
            initial_value=10
        )
        self.at_resource_node_condition = Condition(
            sensor=self.target_step_sensor,
            activator=ThresholdActivator(
                thresholdValue=0,
                isMinimum=False))  # highest activation if the value is below threshold

        # Only go to resource node if we aren't already there
        self.go_to_resource_node_behaviour.add_precondition(
            precondition=Negation(self.at_resource_node_condition))

        # Going to resource has the effect of decreasing the distance to go there
        self.go_to_resource_node_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.target_step_sensor.name,
                indicator=-1.0,  # decreasing by 1 step
                sensor_type=float

            )
        )

    def init_sensors(self, global_rhbp_components):
        """
        Initialise sensors
        :param global_rhbp_components:
        :return:
        """

        self.next_ingredient_volume_sensor = GradientSensor(
            mechanism=global_rhbp_components.gather_decision_mechanism,
            sensor_type=SENSOR.VALUE_ATTRIBUTE,
            attr_name="item.volume",
            initial_value=0,
            name="next_ingredient_volume_sensor"
        )

        self.load_after_next_ingredient_sensor = SubtractionSensor(
            minuend_sensor=self._global_rhbp_components.free_load_sensor,
            subtrahend_sensor=self.next_ingredient_volume_sensor,
            name="load_after_next_ingredient_sensor"
        )
        self.next_item_fits_in_storage_cond = Condition(
            sensor=self.load_after_next_ingredient_sensor,
            activator=ThresholdActivator(thresholdValue=0, isMinimum=True)
        )


    def stop(self):
        """
        When stoping to gather, remove goals
        :return:
        """
        super(GatheringNetworkBehaviour, self).stop()
        # Deleting task and cleaning up goals
        self._global_rhbp_components.gather_decision_mechanism.end_gathering()