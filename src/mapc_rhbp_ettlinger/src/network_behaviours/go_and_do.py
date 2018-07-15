from behaviour_components.activators import ThresholdActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition, Negation
from behaviour_components.goals import GoalBase
from behaviours.movement import GoToDestinationBehaviour
from network_behaviours.battery import BatteryChargingNetworkBehaviour
from rhbp_selforga.gradientsensor import GradientSensor, SENSOR

from sensor.movement import StepDistanceSensor


class GoAndDoNetworkBehaviour(BatteryChargingNetworkBehaviour):
    """
    NetworkBehaviour Base class for all network behaviours, that need to go to a destination and perform ana action there
    """

    def __init__(self, agent_name, name, global_rhbp_components, mechanism, **kwargs):
        super(GoAndDoNetworkBehaviour, self).__init__(
            agent_name=agent_name,
            global_rhbp_components=global_rhbp_components,
            name=name, **kwargs)

        self.destination_decision = mechanism

        self._go_behaviour = None
        self._do_behaviour = None

        self.init_destination_step_sensor()
        self.init_go_behaviour()

    def init_do_behaviour(self, do_behaviour, effect_on_goal=True):
        """
        Initialises a do behaviour. applies the precondition to be at the target position
        :param do_behaviour: The behaviour to apply the precondition on
        :type do_behaviour: BehaviourBase
        :param effect_on_goal:
        :return:
        """


        # Only perform behaviour when at target destination
        self._do_behaviour = do_behaviour

        self._do_behaviour.add_precondition(
            precondition=self.at_destination_cond)

        if effect_on_goal:
            pass
            # TODO: Create goal here to perform task
            # This didnt really work well because it interfered with other goals. Will try that again

    def init_go_behaviour(self):
        """
        Initialises the behaviour, which allows to move to the desired destination
        :return:
        """

        ####################### GO TO DESTINATION BEHAVIOUR ###################
        self._go_behaviour = GoToDestinationBehaviour(
            agent_name=self._agent_name,
            plannerPrefix=self.get_manager_prefix(),
            name=self.get_manager_prefix() + '_go_behaviour',
            mechanism=self.destination_decision
        )

        # Going to Shop gets us 1 step closer to the shop
        self._go_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.target_step_sensor.name,
                indicator=-1.0,
                sensor_type=float))

        # We can only walk to shop until we are there
        self._go_behaviour.add_precondition(
            precondition=Negation(self.at_destination_cond)
        )
        self.apply_charging_restrictions(self._go_behaviour)

    def init_destination_step_sensor(self):
        """
        Initialises the destination step sensor
        :return:
        """

        self.target_sensor = GradientSensor(
            mechanism=self.destination_decision,
            name=self.get_manager_prefix() + "_target_sensor",
            sensor_type=SENSOR.CACHED_VALUE
        )

        # Sensor to check distance to charging station
        self.target_step_sensor = StepDistanceSensor(
            name=self.get_manager_prefix() + "_target_step_sensor",
            agent_name=self._agent_name,
            position_sensor_1=self._global_rhbp_components.agent_position_sensor,
            position_sensor_2=self.target_sensor,
            initial_value=10
        )

        self.at_destination_cond = Condition(
            sensor=self.target_step_sensor,
            activator=ThresholdActivator(
                thresholdValue=0,
                isMinimum=False))
