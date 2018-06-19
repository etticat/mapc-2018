import rospy

from agent_knowledge.tasks import JobKnowledgebase
from agent_knowledge.well import WellTaskKnowledgebase
from behaviour_components.activators import BooleanActivator, ThresholdActivator, LinearActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition, Negation, Disjunction, Conjunction
from behaviour_components.goals import GoalBase
from behaviour_components.network_behavior import NetworkBehaviour
from behaviours.job import GoToStorageBehaviour, AssembleProductBehaviour, GatherBehaviour, GoToResourceBehaviour, \
    GoToWorkshopBehaviour, DeliverJobBehaviour
from behaviours.well import GoToWellBehaviour, WellIntegritySensor, BuildWellBehaviour, \
    BuildUpWellBehaviour
from provider.product_provider import ProductProvider
from rhbp_utils.knowledge_sensors import KnowledgeSensor
from sensor.job import AmountInListActivator, ProductSensor
from sensor.movement import DestinationDistanceSensor


class BuildWellNetworkBehaviour(NetworkBehaviour):

    def __init__(self, agent, name, msg, **kwargs):

        proximity = msg.proximity

        super(BuildWellNetworkBehaviour, self).__init__(name, **kwargs)

        self._product_provider = ProductProvider(
            agent_name=agent._agent_name)

        self.init_assignage_sensor(agent)
        self.init_well_sensors(agent, msg.proximity)

        self.init_go_to_destination_behaviour(agent, proximity)
        self.init_build_behaviour(agent)
        self.init_build_up_behaviour(agent)

        self.requires_action_condition = Conjunction(
            self._has_tasks_assigned_condition,
            self._target_well_damaged_condition
        )

    def init_go_to_destination_behaviour(self, agent, proximity):
        # go to destination
        self.go_to_well_behaviour = GoToWellBehaviour(
            plannerPrefix=self.get_manager_prefix(),
            name="go_to_well",
            agent=agent)

        self.well_distance_sensor = DestinationDistanceSensor(
            name='well_destination_sensor',
            agent_name=agent._agent_name,
            behaviour_name=self.go_to_well_behaviour._name)

        self.go_to_well_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.well_distance_sensor.name,
                indicator=-1.0,
                sensor_type=float
            )
        )

        self.at_well_condition = Condition(
            sensor=self.well_distance_sensor,
            activator=ThresholdActivator(
                thresholdValue=proximity,
                isMinimum=False))  # highest activation if the value is below threshold

        self.go_to_well_behaviour.add_precondition(
            precondition=Negation(self.at_well_condition)
        )



    def init_assignage_sensor(self, agent):
        # Sensor that checks if agent has at least one assigned task
        self.has_tasks_assigned_sensor = KnowledgeSensor(
            name='has_task',
            pattern=WellTaskKnowledgebase.generate_tuple(
                agent_name=agent._agent_name))

        self._has_tasks_assigned_condition = Condition(
            sensor=self.has_tasks_assigned_sensor,
            activator=BooleanActivator(
                desiredValue=True))

    def init_well_sensors(self, agent, proximity):
        # Sensor that checks if agent has at least one assigned task

        self.target_well_integrity_sensor = WellIntegritySensor(
            agent_name=agent._agent_name,
            name="target_well_integrity_sensor",
            proximity=proximity)

        self._target_well_damaged_condition = Condition(
            sensor=self.target_well_integrity_sensor,
            activator=ThresholdActivator(
                thresholdValue=99,
                isMinimum=False
            )
        )

        self._target_well_exists_sensor = Condition(
            sensor=self.target_well_integrity_sensor,
            activator=ThresholdActivator(
                thresholdValue=0,
                isMinimum=True
            )
        )
    def init_build_behaviour(self, agent):
        self.build_well_bahviour = BuildWellBehaviour(
            name="build_well_behaviour",
            agent_name=agent._agent_name,
            plannerPrefix=self.get_manager_prefix()
        )

        self.build_well_bahviour.add_precondition(
            precondition=self.at_well_condition
        )

        self.build_well_bahviour.add_precondition(
            precondition=Negation(self._target_well_exists_sensor)
        )

        self.build_well_bahviour.add_effect(
            effect=Effect(
                sensor_name=self.has_tasks_assigned_sensor.name,
                indicator=-1.0,
                sensor_type=bool
            )
        )

    def init_build_up_behaviour(self, agent):
        self.build_up_well_bahviour = BuildUpWellBehaviour(
            name="build_up_well_behaviour",
            agent_name=agent._agent_name,
            plannerPrefix=self.get_manager_prefix()
        )

        self.build_up_well_bahviour.add_precondition(
            precondition=self.at_well_condition
        )

        self.build_up_well_bahviour.add_precondition(
            precondition=self._target_well_damaged_condition
        )
        self.build_up_well_bahviour.add_precondition(
            precondition=self._target_well_exists_sensor
        )


        self.build_up_well_bahviour.add_effect(
            effect=Effect(
                sensor_name=self.target_well_integrity_sensor.name,
                indicator=1.0,
                sensor_type=float
            )
        )

