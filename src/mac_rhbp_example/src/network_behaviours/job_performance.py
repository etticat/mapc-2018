import rospy
from mac_ros_bridge.msg import Position

from agent_knowledge.facilities import FacilityKnowledgebase
from agent_knowledge.movement import MovementKnowledge
from agent_knowledge.tasks import TaskKnowledge
from behaviour_components.activators import BooleanActivator, ThresholdActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition, Negation
from behaviour_components.goals import GoalBase
from behaviour_components.network_behavior import NetworkBehaviour
from behaviours.generic_action import GenericActionBehaviour, Action
from behaviours.movement import GotoLocationBehaviour
from rhbp_utils.knowledge_sensors import KnowledgeSensor
from sensor.job import IngredientSensor, FinishedProductSensor, AmountInListActivator
from sensor.movement import DestinationDistanceSensor


class JobPerformanceNetwork(NetworkBehaviour):

    def __init__(self, agent, name, msg, **kwargs):

        proximity = msg.proximity

        super(JobPerformanceNetwork, self).__init__(name, **kwargs)


        # Sensor that checks if agent has at least one assigned task
        self.has_tasks_assigned_sensor = KnowledgeSensor(
            name='has_task',
            pattern=TaskKnowledge.get_tuple_task_creation(
                job_id="*",
                task_id="*",
                destination="*",
                agent=agent._agent_name,
                status="assigned"))

        self.has_tasks__assigned_condition= Condition(
            sensor=self.has_tasks_assigned_sensor,
            activator=BooleanActivator(
                desiredValue=True))

        self.has_all_ingredients_sensor = IngredientSensor(
            name="has_all_ingredients_sensor",
           agent_name=agent._agent_name)

        self.has_all_finished_products_sensor = FinishedProductSensor(
            name="has_all_finished_products_sensor",
            agent_name=agent._agent_name)

        self.has_all_ingerdients = Condition(
            sensor=self.has_all_ingredients_sensor,
            activator=AmountInListActivator(
                amount=0
            ))

        self.has_all_products_condition= Condition(
            sensor=self.has_all_finished_products_sensor,
            activator=AmountInListActivator(
                amount=0
            ))

        # go to destination
        self.go_to_destination_behaviour = GoToStorageBehaviour(
            plannerPrefix=self.get_manager_prefix(),
            agent=agent)

        self.go_to_destination_behaviour.add_precondition(
            precondition=self.has_all_products_condition
        )
        self.go_to_destination_behaviour.add_precondition(
            precondition=self.has_all_ingerdients
        )


        self.go_to_resource_node_behaviour = GoToResourceBehaviour(
            agent=agent,
            plannerPrefix=self.get_manager_prefix(),
            ingredientSensor=self.has_all_ingredients_sensor)


        resource_destination_sensor = DestinationDistanceSensor(
            name='resource_destination_sensor',
            agent_name=agent._agent_name,
            behaviour_name=self.go_to_resource_node_behaviour._name)

        self.go_to_resource_node_behaviour.add_effect(
            effect=Effect(
                sensor_name=resource_destination_sensor.name,
                indicator=-1.0,
                sensor_type=float

            )
        )

        at_resource_node_condition = Condition(
            sensor=resource_destination_sensor,
            activator=ThresholdActivator(
                thresholdValue=proximity,
                isMinimum=False))  # highest activation if the value is below threshold

        #TODO Leave this behaviour again when enough is gathered
        self.gather_ingredients_behaviour = GatherBehaviour(name="gather_behaviour", agent_name=agent._agent_name, plannerPrefix=self.get_manager_prefix())

        self.gather_ingredients_behaviour.add_precondition(at_resource_node_condition)

        self.gather_ingredients_behaviour.add_effect(Effect(
            sensor_name=self.has_all_ingredients_sensor.name,
            indicator=1.0,
            sensor_type=bool)
        ) # TODO: this will be changed to int


        # TODO: assemble finished products
        # TODO: Request help from others
        self.assemble_product_behaviour = None

        # Goal is to finish all tasks
        self._job_performance_goal = GoalBase(
            name='job_performance',
            permanent=True,
            plannerPrefix=agent._agent_name,
            conditions=[Negation(self.has_tasks__assigned_condition)])




class GoToResourceBehaviour(GotoLocationBehaviour):
    def __init__(self, agent,plannerPrefix, ingredientSensor, **kwargs):
        super(GoToResourceBehaviour, self).__init__(
            agent._agent_name,
            plannerPrefix=plannerPrefix,
            name="go_to_resource",
            **kwargs)
        self._selected_facility = None
        self.ingredientSensor = ingredientSensor # TODO remove this and just extract the needed code
        self.taskKnowledge = TaskKnowledge(agent._agent_name)
        self.facility_knowledge = FacilityKnowledgebase()

    def _select_pos(self):
        ingredientItem = self.ingredientSensor.get_still_needed_items()[0]
        facilityPos = self.facility_knowledge.get_resource_map()[list(ingredientItem.keys())[0]]

        return facilityPos

class GoToStorageBehaviour(GotoLocationBehaviour):

    def __init__(self, agent,plannerPrefix,  **kwargs):
        super(GoToStorageBehaviour, self).__init__(
            agent._agent_name,
            plannerPrefix=plannerPrefix,
            name="go_to_storage",
            graph_name="storage",
            **kwargs)
        self._selected_facility = None
        self.taskKnowledge = TaskKnowledge(agent._agent_name)
        self.facility_knowledge = FacilityKnowledgebase()

    def _select_pos(self):
        task = self.taskKnowledge.get_task(agent_name=self._agent_name, status="assigned")
        facility = self.facility_knowledge.storages[task.destination]
        return facility.pos


class GatherBehaviour(GenericActionBehaviour):
    """
    A behaviour for triggering charge actions which can be executed everywhere
    """

    def __init__(self, name, agent_name, **kwargs):
        """
        :param name: name of the behaviour
        :param agent_name: name of the agent for determining the correct topic prefix
        :param kwargs: more optional parameter that are passed to the bas class
        """
        super(GatherBehaviour, self) \
            .__init__(name=name,
                      agent_name = agent_name,
                      action_type = Action.GATHER,
                      **kwargs)
        self._movement_knowledge = MovementKnowledge(behaviour_name="go_to_resource", agent_name=agent_name)

    def stop(self):
        self._movement_knowledge.stop_movement()
        super(GatherBehaviour, self).stop()