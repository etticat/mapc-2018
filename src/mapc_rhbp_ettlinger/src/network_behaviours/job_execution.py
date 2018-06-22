from agent_knowledge.tasks import JobKnowledgebase
from behaviour_components.activators import BooleanActivator, ThresholdActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition, Negation
from behaviour_components.goals import GoalBase
from behaviour_components.network_behavior import NetworkBehaviour
from behaviours.job import GoToStorageForJobBehaviour, DeliverJobBehaviour
from provider.product_provider import ProductProvider
from rhbp_utils.knowledge_sensors import KnowledgeSensor
from sensor.movement import DestinationDistanceSensor


class JobExecutionNetworkBehaviour(NetworkBehaviour):

    def __init__(self, agent, name, msg, **kwargs):

        proximity = msg.proximity

        super(JobExecutionNetworkBehaviour, self).__init__(name, **kwargs)

        self._product_provider = ProductProvider(
            agent_name=agent._agent_name)

        self.init_task_sensor(agent)

        self.init_go_to_destination_behaviour(agent, proximity)
        self.init_deliver_job_behaviour(agent)

        self.goal = GoalBase(
            name='job_fulfillment_goal',
            permanent=True,
            plannerPrefix=self.get_manager_prefix(),
            conditions=[Negation(self.has_tasks_assigned_condition)])


    def init_go_to_destination_behaviour(self, agent, proximity):
        # go to destination
        self.go_to_destination_behaviour = GoToStorageForJobBehaviour(
            plannerPrefix=self.get_manager_prefix(),
            name="go_to_storage",
            agent=agent)

        self.storage_destination_sensor = DestinationDistanceSensor(
            name='storage_destination_sensor',
            agent_name=agent._agent_name,
            behaviour_name=self.go_to_destination_behaviour._name)

        self.go_to_destination_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.storage_destination_sensor.name,
                indicator=-1.0,
                sensor_type=float
            )
        )


        self.at_storage_condition = Condition(
            sensor=self.storage_destination_sensor,
            activator=ThresholdActivator(
                thresholdValue=proximity,
                isMinimum=False))  # highest activation if the value is below threshold


        self.go_to_destination_behaviour.add_precondition(
            precondition=Negation(self.at_storage_condition)
        )



    def init_task_sensor(self, agent):
        # Sensor that checks if agent has at least one assigned task
        self.has_tasks_assigned_sensor = KnowledgeSensor(
            name='has_task',
            pattern=JobKnowledgebase.generate_tuple(
                agent_name=agent._agent_name))
        self.has_tasks_assigned_condition = Condition(
            sensor=self.has_tasks_assigned_sensor,
            activator=BooleanActivator(
                desiredValue=True))

    def init_deliver_job_behaviour(self, agent):
        self.deliver_job_bahviour = DeliverJobBehaviour(
            name="deliver_job_behaviour",
            agent_name=agent._agent_name,
            plannerPrefix=self.get_manager_prefix()
        )

        self.deliver_job_bahviour.add_precondition(
            precondition=self.at_storage_condition
        )

        self.deliver_job_bahviour.add_effect(
            effect=Effect(
                sensor_name=self.has_tasks_assigned_sensor.name,
                indicator=-1.0,
                sensor_type=bool
            )
        )

