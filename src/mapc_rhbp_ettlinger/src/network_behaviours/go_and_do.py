from behaviour_components.activators import ThresholdActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition, Negation
from behaviours.movement import GoToDestinationBehaviour
from network_behaviours.battery import BatteryChargingNetworkBehaviour
from rhbp_selforga.gradientsensor import GradientSensor, SENSOR

from sensor.movement import StepDistanceSensor


class GoAndDoNetworkBehaviour(BatteryChargingNetworkBehaviour):
    """
    NetworkBehaviour Base class for all network behaviours, that need to go to a destination and perform ana action
    there
    """

    def __init__(self, agent_name, name, shared_components, mechanism, use_in_facility_flag=True,
                 recalculate_destination_every_step=False, use_name_for_movement=False, **kwargs):
        super(GoAndDoNetworkBehaviour, self).__init__(
            agent_name=agent_name,
            shared_components=shared_components,
            name=name, **kwargs)

        self.destination_decision = mechanism
        self.use_name_for_movement = use_name_for_movement
        self._recalculate_destination_every_step = recalculate_destination_every_step

        self._go_behaviour = None
        self._do_behaviour = None

        self.init_destination_step_sensor(use_in_facility_flag)
        self.init_go_behaviour()
        self.init_go_and_do_charing_restrictions()

    def init_do_behaviour(self, do_behaviour):
        """
        Initialises a do behaviour. applies the precondition to be at the target position
        :param do_behaviour: The behaviour to apply the precondition on
        :type do_behaviour: BehaviourBase
        :return:
        """

        # Only perform behaviour when at target destination
        self._do_behaviour = do_behaviour

        self._do_behaviour.add_precondition(
            precondition=self.at_destination_cond)


    def init_go_behaviour(self):
        """
        Initialises the behaviour, which allows to move to the desired destination
        :return:
        """

        ####################### GO TO DESTINATION BEHAVIOUR ###################
        self._go_behaviour = GoToDestinationBehaviour(
            agent_name=self._agent_name,
            plannerPrefix=self.get_manager_prefix(),
            recalculate_destination_every_step=self._recalculate_destination_every_step,
            use_name_for_movement=self.use_name_for_movement,
            name=self.get_manager_prefix() + '_go_behaviour',
            mechanism=self.destination_decision
        )

        # Going to Shop gets us 1 step closer to the shop
        self._go_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.target_step_sensor.name,
                indicator=-1.0,
                sensor_type=float))

        # Worst case scenario: By moving we move 1 step away from a charging station
        self._go_behaviour.add_effect(
            Effect(
                sensor_name=self._shared_components.charging_station_step_sensor.name,
                indicator=1.0,  # 1 step at a time
                sensor_type=float))

        # We can only walk to shop until we are there
        self._go_behaviour.add_precondition(
            precondition=Negation(self.at_destination_cond)
        )
        self.apply_charging_restrictions(self._go_behaviour)

    def init_destination_step_sensor(self, use_in_facility_flag):
        """
        Initialises the destination step sensor
        :return:
        """

        if self._recalculate_destination_every_step:
            target_sensor_type = SENSOR.VALUE
        else:
            target_sensor_type = SENSOR.CACHED_VALUE

        self.target_sensor = GradientSensor(
            mechanism=self.destination_decision,
            name=self.get_manager_prefix() + "_target_sensor",
            sensor_type=target_sensor_type
        )

        # Sensor to check distance to charging station
        self.target_step_sensor = StepDistanceSensor(
            name=self.get_manager_prefix() + "_target_step_sensor",
            agent_name=self._agent_name,
            destination_sensor=self.target_sensor,
            use_in_facility_flag=use_in_facility_flag,
            initial_value=10
        )

        self.at_destination_cond = Condition(
            sensor=self.target_step_sensor,
            activator=ThresholdActivator(
                thresholdValue=0,
                isMinimum=False))

    def init_go_and_do_charing_restrictions(self):
        self._go_to_charging_station_behaviour.add_precondition(
            Negation(self.at_destination_cond))
        self._recharge_behaviour.add_precondition(
            Negation(self.at_destination_cond))
