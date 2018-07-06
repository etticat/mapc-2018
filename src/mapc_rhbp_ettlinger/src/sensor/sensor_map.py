import rospy
from mac_ros_bridge.msg import Agent

from behaviour_components.activators import ThresholdActivator, LinearActivator, BooleanActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition, Negation, Disjunction
from behaviour_components.sensors import TopicSensor
from common_utils.agent_utils import AgentUtils
from decisions.battery import ClosestChargingStationDecision
from decisions.choose_stroage_for_hoarding import ChooseStorageMechanism
from decisions.gather import ChooseResourceMechanism
from decisions.p_task_decision import CurrentTaskDecision
from rhbp_selforga.gradientsensor import GradientSensor, SENSOR
from rhbp_utils.knowledge_sensors import KnowledgeSensor
from sensor.agent import FinishedProductLoadSensor
from sensor.exploration import ResourceDiscoveryProgressSensor, DiscoveryProgressSensor, OldestCellLastSeenSensor
from sensor.gather import SmallestGatherableItemSensor
from sensor.general import FactorSensor, SubtractionSensor
from sensor.movement import StepDistanceSensor


class SensorAndConditionMap(object):

    DISCOVERY_AGE_FULL_ACTIVATION = 100

    def __init__(self, agent_name):
        self.agent_name = agent_name
        self.agent_topic = AgentUtils.get_bridge_topic_agent(agent_name=agent_name)
        self.init_load_sensors()
        self.init_agent_sensors()
        self.init_battery_sensors()
        self.init_resource_sensor(agent_name=agent_name)
        self.init_task_sensor(agent_name=agent_name)
        self.init_exploration_sensor(agent_name=agent_name)

        rospy.Subscriber(AgentUtils.get_bridge_topic_agent(self.agent_name), Agent, self.callback_agent)

    def init_load_sensors(self):
        self.max_load_sensor = TopicSensor(
            topic=self.agent_topic,
            name="max_load_sensor",
            message_attr='load_max')
        self.load_sensor = TopicSensor(
            topic=self.agent_topic,
            name="load_sensor",
            message_attr='load')

        self.finished_product_load_sensor = FinishedProductLoadSensor(
            agent_name=self.agent_name,
            name="finished_product_load_sensor")

        self.free_load_sensor = SubtractionSensor(
            name="free_load_sensor",
            minuend_sensor=self.max_load_sensor,
            subtrahend_sensor=self.load_sensor
        )

        self.free_load_sensor = SubtractionSensor(
            name="free_load_sensor",
            minuend_sensor=self.max_load_sensor,
            subtrahend_sensor=self.load_sensor
        )
        self.load_factor_sensor = FactorSensor(
            name="load_factor_sensor",
            dividend_sensor=self.load_sensor,
            divisor_sensor=self.max_load_sensor
        )

        self.load_fullness_condition = Condition(
            name="load_fullness_condition",
            sensor=self.load_factor_sensor,
            activator=LinearActivator(
                zeroActivationValue=0.0,
                fullActivationValue=1.0
            )
        )
        self.finished_product_load_factor_sensor = FactorSensor(
            name="finished_product_load_factor_sensor",
            dividend_sensor=self.finished_product_load_sensor,
            divisor_sensor=self.max_load_sensor
        )

        self.finished_product_load_fullness_condition = Condition(
            name="finished_product_load_fullness_condition",
            sensor=self.finished_product_load_factor_sensor,
            activator=LinearActivator(
                zeroActivationValue=0.0,
                fullActivationValue=1.0
            )
        )
        self.has_finished_products_cond = Condition(
            name="has_finished_products_cond",
            sensor=self.finished_product_load_sensor,
            activator=ThresholdActivator(
                isMinimum=True,
                thresholdValue=1)
        )
        self.smallest_gatherable_item_sensor = SmallestGatherableItemSensor(
            name="smallest_gatherable_item_sensor",
            agent_name=self.agent_name
        )

        self.load_after_next_gathering_sensor = SubtractionSensor(
            name="load_after_next_gathering_sensor",
            minuend_sensor=self.free_load_sensor,
            subtrahend_sensor=self.smallest_gatherable_item_sensor,
        )

        self.can_fit_more_ingredients_cond = Condition(
            sensor=self.load_after_next_gathering_sensor,
            activator=ThresholdActivator(
                isMinimum=True,
                thresholdValue=0
            )
        )

    def init_agent_sensors(self):
        self.agent_position_sensor = TopicSensor(
            topic=self.agent_topic,
            name="agent_position_sensor",
            message_attr='pos')

    def init_battery_sensors(self):
        agent_topic = AgentUtils.get_bridge_topic_agent(self.agent_name)

        self.closest_charging_station_decision = ClosestChargingStationDecision(agent_name=self.agent_name)

        self.closest_charging_station_sensor = GradientSensor(
            name="closest_charging_station_sensor",
            mechanism=self.closest_charging_station_decision,
            sensor_type=SENSOR.VALUE)

        # Sensor to check distance to charging station
        self.charging_station_step_sensor = StepDistanceSensor(
            name='charging_station_step_distance',
            position_sensor_1=self.agent_position_sensor,
            position_sensor_2=self.closest_charging_station_sensor,
            initial_value=10
        )

        # Sensor that checks if vehicles is charged at the moment
        self.charge_sensor = TopicSensor(
            topic=agent_topic,
            name="charge_sensor",
            message_attr='charge')

        # charging required condition: When closer to lower bound -> higher activation
        self._require_charge_activator = LinearActivator(zeroActivationValue=15, fullActivationValue=4)
        self.require_charging_cond = Condition(
            sensor=self.charge_sensor,
            activator=self._require_charge_activator)  # highest activation already before battery empty

        # Condition to check if we are at a charging station
        self.at_charging_station_cond = Condition(
            sensor=self.charging_station_step_sensor,
            activator=ThresholdActivator(
                thresholdValue=0,
                isMinimum=False))

        # battery is completely empty
        self.battery_empty_cond = Condition(
            sensor=self.charge_sensor,
            activator=ThresholdActivator(
                thresholdValue=0,
                isMinimum=False))

        # CONDITION: Vehicle has enough charge to function
        self.enough_battery_to_move_cond = Condition(
            sensor=self.charge_sensor,
            activator=ThresholdActivator(
                thresholdValue=1,
                isMinimum=True))

        # Movement decreases charge
        self.go_to_charging_station_effect = \
            Effect(
                sensor_name=self.charge_sensor.name,
                sensor_type=float,
                indicator=-1.0)

        # Recharging has effect on charge_behaviour sensor
        self.recharge_effect = Effect(
            sensor_name=self.charge_sensor.name,
            indicator=1.0,
            sensor_type=float)

        self.charge_behaviour_effect = Effect(
            sensor_name=self.charge_sensor.name,
            indicator=100,  # here we could also use the real charging effects
            # this is replaced immediately by the subscriber callback
            sensor_type=float)

    def init_resource_sensor(self, agent_name):
        self.resource_discovery_progress_sensor = ResourceDiscoveryProgressSensor(
            name="resource_discovery_progress_sensor",
            agent_name=agent_name

        )
        self.resource_discovery_completeness_cond = Condition(
            sensor=self.resource_discovery_progress_sensor,
            activator=LinearActivator(
                zeroActivationValue=0.0,
                fullActivationValue=1.0
            ))
        self.resources_of_all_items_discovered_condition = Condition(
            sensor=self.resource_discovery_progress_sensor,
            activator=ThresholdActivator(
                thresholdValue=1.0,
                isMinimum=True))

    def init_task_sensor(self, agent_name):

        self.assemble_task_mechanism = CurrentTaskDecision(agent_name=agent_name, task_type=CurrentTaskDecision.TYPE_ASSEMBLE)
        self.deliver_task_mechanism = CurrentTaskDecision(agent_name=agent_name, task_type=CurrentTaskDecision.TYPE_DELIVER)
        self.well_task_mechanism = CurrentTaskDecision(agent_name=agent_name, task_type=CurrentTaskDecision.TYPE_BUILD_WELL)
        # TODO: GATHER
        self.choose_resource_mechanism = ChooseResourceMechanism(agent_name=agent_name)
        self.choose_hoarding_mechanism = ChooseStorageMechanism(agent_name=agent_name)

        self.assemble_task_sensor = GradientSensor(
            name="assemble_task_sensor",
            sensor_type=SENSOR.VALUE,
            mechanism=self.assemble_task_mechanism
        )

        self.has_assemble_task_sensor = GradientSensor(
            name="has_assemble_task_sensor",
            sensor_type=SENSOR.VALUE_EXISTS,
            mechanism=self.assemble_task_mechanism
        )

        self.has_assemble_task_assigned_cond = Condition(
            name="has_assemble_task_assigned_cond",
            sensor=self.has_assemble_task_sensor,
            activator=BooleanActivator(desiredValue=True)
        )

        self.well_task_sensor = GradientSensor(
            name="well_task_sensor",
            sensor_type=SENSOR.VALUE,
            mechanism=self.well_task_mechanism
        )

        self.has_well_task_sensor = GradientSensor(
            name="has_well_task_sensor",
            sensor_type=SENSOR.VALUE_EXISTS,
            mechanism=self.well_task_mechanism
        )

        self.has_build_well_task_assigned_cond = Condition(
            name="has_build_well_task_assigned_cond",
            sensor=self.has_well_task_sensor,
            activator=BooleanActivator(desiredValue=True)
        )

        self.deliver_task_sensor = GradientSensor(
            name="deliver_task_sensor",
            sensor_type=SENSOR.VALUE,
            mechanism=self.deliver_task_mechanism
        )
        self.has_deliver_task_sensor = GradientSensor(
            name="has_deliver_task_sensor",
            sensor_type=SENSOR.VALUE_EXISTS,
            mechanism=self.deliver_task_mechanism
        )
        self.has_deliver_job_task_assigned_cond = Condition(
            name="has_deliver_job_task_assigned_cond",
            sensor=self.has_deliver_task_sensor,
            activator=BooleanActivator(desiredValue=True)
        )

        self.has_task_assigned_cond = Disjunction(
            self.has_assemble_task_assigned_cond,
            self.has_deliver_job_task_assigned_cond,
            self.has_build_well_task_assigned_cond
        )
        self.has_no_task_assigned_cond = Negation(
            self.has_task_assigned_cond
        )

    def callback_agent(self, msg):
        """

        :param msg:
        :type msg: Agent
        :return:
        """
        agent_charge_upper_bound = msg.charge_max # This can change on upgrade!

        agent_charge_lower_bound = agent_charge_upper_bound * 0.2

        self._require_charge_activator.fullActivationValue = agent_charge_lower_bound
        self._require_charge_activator.zeroActivationValue = agent_charge_upper_bound

    def init_exploration_sensor(self, agent_name):
        self.discovery_progress_sensor = DiscoveryProgressSensor(
            agent_name=agent_name,
            name="discovery_completeness_condition")


        self.discovery_completeness_cond = Condition(
            sensor=self.discovery_progress_sensor,
            activator=LinearActivator(
                zeroActivationValue=0.0,
                fullActivationValue=1.0
            ))

        self.oldest_cell_last_seen_sensor = OldestCellLastSeenSensor(
            agent_name=agent_name,
            initial_value=-SensorAndConditionMap.DISCOVERY_AGE_FULL_ACTIVATION,
            name="discovery_completeness_condition")

        self.simulation_step_sensor = TopicSensor(
            topic=AgentUtils.get_bridge_topic(agent_name=agent_name, postfix="request_action"),
            name="simulation_step_sensor",
            message_attr='simulation_step')

        self.oldest_cell_age_sensor = SubtractionSensor(
            minuend_sensor=self.simulation_step_sensor,
            subtrahend_sensor=self.oldest_cell_last_seen_sensor,
            name="oldest_cell_age_sensor"
        )

        self.oldest_cell_requires_exploration = Condition(
            sensor=self.oldest_cell_age_sensor,
            activator=LinearActivator(
                zeroActivationValue=0,
                fullActivationValue=SensorAndConditionMap.DISCOVERY_AGE_FULL_ACTIVATION
            )
        )