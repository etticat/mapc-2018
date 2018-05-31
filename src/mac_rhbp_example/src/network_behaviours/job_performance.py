import rospy
from mac_ros_bridge.msg import Position

from agent_common.job_utils import JobUtils
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


        # TODO: Do I need these #33-1
        # Seems to work without. With them I get errors but it continues to run normally
        # Goal is to finish all tasks
        # self._job_performance_goal = GoalBase(
        #     name='job_performance',
        #     permanent=True,
        #     plannerPrefix=agent._agent_name,
        #     conditions=[Negation(self.has_tasks__assigned_condition)])




class GoToResourceBehaviour(GotoLocationBehaviour):
    def __init__(self, agent,plannerPrefix, **kwargs):
        super(GoToResourceBehaviour, self).__init__(
            agent._agent_name,
            plannerPrefix=plannerPrefix,
            name="go_to_resource",
            **kwargs)
        self._selected_facility = None
        self.taskKnowledge = TaskKnowledge(agent._agent_name)
        self.facility_knowledge = FacilityKnowledgebase()

    def _select_pos(self):
        neededIngredients = self.taskKnowledge.get_required_ingredients(self._agent_name)
        if len(neededIngredients) > 0:
            # TODO: Select the next ingredient to gather somehow smarter. Like going to the closest one
            for neededIngredient in neededIngredients:
                facilityPos = self.facility_knowledge.get_resources(neededIngredient)
                if len(facilityPos)>0:
                    # TODO: Select this smarter. Maybe the closest one
                    self._selected_destination = neededIngredient
                    return facilityPos[0]

        else:
            rospy.logerr("%s:: Trying to go to resource but there is no ingredient left to gather.", self.name)

        rospy.logerr("%s:: Can't find resource node for any required ingredient. %s", self.name, str(neededIngredients))
        self._selected_destination = "none"
        return None

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
        task = self.taskKnowledge.get_tasks(agent_name=self._agent_name, status="assigned")
        # Goint to any destination
        # TODO: Check this. The goal is to only take tasks with the same destination, so this should be fine.
        # Maybe this can be done more elegant
        if len(task) > 0:
            return self.facility_knowledge.storages[task.destination].pos


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
        self._task_knowledge = TaskKnowledge(agent_name=agent_name)

    def do_step(self):

        required_ingredients = self._task_knowledge.get_required_ingredients(self._agent_name)
        fact = self._movement_knowledge.get_current_fact()

        # If we don't need item anymore stop gathering
        # TODO: This could be moved to a seperate sensor
        if fact[MovementKnowledge.INDEX_MOVEMENT_DESTINATION] not in required_ingredients:
            self._movement_knowledge.stop_movement()

        super(GatherBehaviour, self).do_step()