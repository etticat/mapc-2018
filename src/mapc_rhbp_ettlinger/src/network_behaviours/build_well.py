from behaviour_components.activators import ThresholdActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition, Negation
from behaviours.well import BuildWellBehaviour, BuildUpWellBehaviour
from common_utils import etti_logging
from network_behaviours.go_and_do import GoAndDoNetworkBehaviour
from sensor.well import WellIntegritySensor

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.network.well')


class BuildWellNetworkBehaviour(GoAndDoNetworkBehaviour):
    """
    Network Behaviour responsible for building wells
    """

    def __init__(self, agent_name, name, global_rhbp_components, **kwargs):

        super(BuildWellNetworkBehaviour, self).__init__(
            agent_name=agent_name,
            global_rhbp_components=global_rhbp_components,
            name=name,
            mechanism=global_rhbp_components.well_task_mechanism,
            **kwargs)

        self._agent_name = agent_name

        self.init_well_sensors(global_rhbp_components)
        self.init_build_behaviour(global_rhbp_components)
        self.init_build_up_behaviour(global_rhbp_components)

    def init_well_sensors(self, global_rhbp_components):
        """
        Initialise well sensors
        :param global_rhbp_components:
        :return:
        """

        # Sensor to check the integrity of the target well
        self.target_well_integrity_sensor = WellIntegritySensor(
            agent_name=self._agent_name,
            mechanism=global_rhbp_components.well_task_mechanism,
            name="target_well_integrity_sensor")

        self._target_well_intact_condition = Condition(
            sensor=self.target_well_integrity_sensor,
            activator=ThresholdActivator(
                thresholdValue=1,
                isMinimum=True
            )
        )

        self._target_well_damaged_condition = Negation(self._target_well_intact_condition)

        self._target_well_exists_cond = Condition(
            sensor=self.target_well_integrity_sensor,
            activator=ThresholdActivator(
                thresholdValue=0,
                isMinimum=True
            )
        )

    def init_build_behaviour(self, global_rhbp_components):
        """
        Initialise build well behaviour
        :param global_rhbp_components:
        :return:
        """
        self.build_well_behaviour = BuildWellBehaviour(
            name="build_well_behaviour",
            agent_name=self._agent_name,
            plannerPrefix=self.get_manager_prefix(),
            mechanism=global_rhbp_components.well_task_mechanism
        )

        self.build_well_behaviour.add_precondition(
            precondition=Negation(self._target_well_exists_cond)
        )
        self.build_well_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.target_well_integrity_sensor.name,
                indicator=1.0,
                sensor_type=float

            )
        )

        self.init_do_behaviour(self.build_well_behaviour, effect_on_goal=False)

    def init_build_up_behaviour(self, global_rhbp_components):
        """
        Initialise build up well behaviour
        :param global_rhbp_components:
        :return:
        """

        self.build_up_well_bahviour = BuildUpWellBehaviour(
            name="build_up_well_behaviour",
            agent_name=self._agent_name,
            plannerPrefix=self.get_manager_prefix(),
            mechanism=global_rhbp_components.well_task_mechanism
        )

        self.build_up_well_bahviour.add_precondition(
            precondition=self._target_well_damaged_condition
        )
        self.build_up_well_bahviour.add_precondition(
            precondition=self._target_well_exists_cond
        )

        self.build_well_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.target_well_integrity_sensor.name,
                indicator=1.0,
                sensor_type=float

            )
        )

        self.init_do_behaviour(self.build_up_well_bahviour, effect_on_goal=False)
