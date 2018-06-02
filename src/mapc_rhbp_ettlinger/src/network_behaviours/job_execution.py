import rospy

from agent_knowledge.tasks import TaskKnowledgebase
from behaviour_components.activators import BooleanActivator, ThresholdActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition, Negation
from behaviour_components.network_behavior import NetworkBehaviour
from behaviours.job import GoToStorageBehaviour, AssembleProductBehaviour, GatherBehaviour, GoToResourceBehaviour, \
    GoToWorkshopBehaviour, DeliverJobBehaviour
from rhbp_utils.knowledge_sensors import KnowledgeSensor
from sensor.job import IngredientSensor, FinishedProductSensor, AmountInListActivator
from sensor.movement import DestinationDistanceSensor


class JobExecutionNetworkBehaviour(NetworkBehaviour):

    def __init__(self, agent, name, msg, **kwargs):

        proximity = msg.proximity

        super(JobExecutionNetworkBehaviour, self).__init__(name, **kwargs)

        self.init_task_sensor(agent)
        self.init_ingredient_sensor(agent)
        self.init_finished_product_sensor(agent)

        self.init_go_to_resource_behaviour(agent, proximity)
        self.init_gather_behaviour(agent)
        self.init_go_to_workshop_behaviour(agent,proximity)
        self.init_assembly_behaviour(agent)
        self.init_go_to_destinatino_behaviour(agent, proximity)
        self.init_deliver_job_behaviour(agent)




        # TODO: Do I need these #33-1
        # Seems to work without. With them I get errors but it continues to run normally
        # Goal is to finish all tasks
        # self._job_performance_goal = GoalBase(
        #     name='job_performance',
        #     permanent=True,
        #     plannerPrefix=agent._agent_name,
        #     conditions=[Negation(self.has_tasks__assigned_condition)])

    def init_go_to_destinatino_behaviour(self, agent, proximity):
        # go to destination
        self.go_to_destination_behaviour = GoToStorageBehaviour(
            plannerPrefix=self.get_manager_prefix(),
            name="go_to_storage",
            agent=agent)

        self.go_to_destination_behaviour.add_precondition(
            precondition=self.has_all_ingredients_condition
        )

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

        self.go_to_destination_behaviour.add_precondition(
            precondition=self.has_all_products_condition
        )

        self.at_storage_condition = Condition(
            sensor=self.storage_destination_sensor,
            activator=ThresholdActivator(
                thresholdValue=proximity,
                isMinimum=False))  # highest activation if the value is below threshold


        self.go_to_destination_behaviour.add_precondition(
            precondition=Negation(self.at_storage_condition)
        )


    def init_assembly_behaviour(self, agent):
        self.assemble_product_behaviour = AssembleProductBehaviour(
            name="assemble_product_behaviour",
            agent_name=agent._agent_name,
            plannerPrefix=self.get_manager_prefix()
        )
        self.assemble_product_behaviour.add_precondition(
            precondition=self.has_all_ingredients_condition)
        self.assemble_product_behaviour.add_precondition(
            precondition=Negation(self.has_all_products_condition)
        )
        self.assemble_product_behaviour.add_precondition(
            precondition=self.at_workshop_condition
        )
        self.add_effect(
            effect=Effect(
                sensor_name=self.has_all_finished_products_sensor.name,
                indicator=1.0,
                sensor_type=bool
            )
        )

    def init_gather_behaviour(self, agent):
        self.gather_ingredients_behaviour = GatherBehaviour(
            name="gather_behaviour",
            agent_name=agent._agent_name,
            plannerPrefix=self.get_manager_prefix(),
            behaviour_name=self.go_to_resource_node_behaviour.name)

        self.gather_ingredients_behaviour.add_precondition(
            precondition=self.at_resource_node_condition)

        self.gather_ingredients_behaviour.add_effect(Effect(
            sensor_name=self.has_all_ingredients_sensor.name,
            indicator=1.0,
            sensor_type=bool)
        )  # TODO: this will be changed to int

    def init_go_to_resource_behaviour(self, agent, proximity):
        self.go_to_resource_node_behaviour = GoToResourceBehaviour(
            agent=agent,
            name="go_to_resource_for_job",
            plannerPrefix=self.get_manager_prefix())

        self.go_to_resource_node_behaviour.add_precondition(
            precondition=Negation(self.has_all_ingredients_condition)
        )

        self.resource_destination_sensor = DestinationDistanceSensor(
            name='resource_destination_sensor',
            agent_name=agent._agent_name,
            behaviour_name=self.go_to_resource_node_behaviour._name)

        self.at_resource_node_condition = Condition(
            sensor=self.resource_destination_sensor,
            activator=ThresholdActivator(
                thresholdValue=proximity,
                isMinimum=False))  # highest activation if the value is below threshold


        self.go_to_resource_node_behaviour.add_precondition(
            precondition=Negation(self.at_resource_node_condition))


    def init_finished_product_sensor(self, agent):
        self.has_all_finished_products_sensor = FinishedProductSensor(
            name="has_all_finished_products_sensor",
            agent_name=agent._agent_name)
        self.has_all_products_condition = Condition(
            sensor=self.has_all_finished_products_sensor,
            activator=AmountInListActivator(
                amount=0
            ))

    def init_ingredient_sensor(self, agent):
        self.has_all_ingredients_sensor = IngredientSensor(
            name="has_all_ingredients_sensor",
            agent_name=agent._agent_name)
        self.has_all_ingredients_condition = Condition(
            sensor=self.has_all_ingredients_sensor,
            activator=AmountInListActivator(
                amount=0
            ))

    def init_task_sensor(self, agent):
        # Sensor that checks if agent has at least one assigned task
        self.has_tasks_assigned_sensor = KnowledgeSensor(
            name='has_task',
            pattern=TaskKnowledgebase.generate_tuple(
                job_id="*",
                task_id="*",
                destination="*",
                agent=agent._agent_name,
                status="assigned"))
        self.has_tasks__assigned_condition = Condition(
            sensor=self.has_tasks_assigned_sensor,
            activator=BooleanActivator(
                desiredValue=True))

    def init_go_to_workshop_behaviour(self, agent, proximity):
        # go to destination
        self.go_to_workshop_behaviour = GoToWorkshopBehaviour(
            name="go_to_workshop_behaviour",
            plannerPrefix=self.get_manager_prefix(),
            agent=agent,
            agent_name=agent._agent_name,
            topic="/workshop"
        )

        self.go_to_workshop_behaviour.add_precondition(
            precondition=self.has_all_ingredients_condition
        )
        self.go_to_workshop_behaviour.add_precondition(
            precondition=Negation(self.has_all_products_condition
            )
        )

        self.workshop_destination_sensor = DestinationDistanceSensor(
            name='workshop_destination_sensor',
            agent_name=agent._agent_name,
            behaviour_name=self.go_to_workshop_behaviour._name)

        self.go_to_workshop_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.workshop_destination_sensor.name,
                indicator=-1.0,
                sensor_type=float
            )
        )

        self.at_workshop_condition = Condition(
            sensor=self.workshop_destination_sensor,
            activator=ThresholdActivator(
                thresholdValue=proximity,
                isMinimum=False))  # highest activation if the value is below threshold

    def init_deliver_job_behaviour(self, agent):
        self.deliver_job_bahviour = DeliverJobBehaviour(
            name="deliver_job_bahviour",
            agent_name=agent._agent_name,
            plannerPrefix=self.get_manager_prefix()
        )
        self.deliver_job_bahviour.add_precondition(
            precondition=self.has_all_products_condition
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

