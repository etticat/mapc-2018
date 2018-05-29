from mac_ros_bridge.msg import Position

from agent_knowledge.facilities import FacilityKnowledgebase
from agent_knowledge.tasks import TaskKnowledge
from behaviour_components.activators import BooleanActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition, Negation
from behaviour_components.goals import GoalBase
from behaviour_components.network_behavior import NetworkBehaviour
from behaviours.movement import GotoLocationBehaviour
from rhbp_utils.knowledge_sensors import KnowledgeSensor


class JobPerformanceNetwork(NetworkBehaviour):

    def __init__(self, agent, name, **kwargs):

        super(JobPerformanceNetwork, self).__init__(name, **kwargs)


        # Sensor that checks if agent has at least one assigned task
        has_tasks_assigned_sensor = KnowledgeSensor(
            name='has_task',
            pattern=TaskKnowledge.get_tuple_task_creation(
                job_id="*",
                task_id="*",
                destination="*",
                agent=agent._agent_name,
                status="assigned"))

        self.has_tasks__assigned_condition= Condition(
            sensor=has_tasks_assigned_sensor,
            activator=BooleanActivator(
                desiredValue=True))

        # TODO: Check if we hold all the ingredients neccessary to perform all tasks
        self.has_all_ingredients_sensor = None

        # TODO: Check if we hold all finished products
        self.has_all_ingredients_sensor = None

        # TODO: If we need further ingredients
        self.go_to_resource_node_behaviour = None

        # go to destination
        self.go_to_destination_behaviour = GoToStorageBehaviour(
            plannerPrefix=agent._agent_name,
            agent=agent)
        self.go_to_destination_behaviour.add_precondition(
            precondition=self.has_tasks__assigned_condition)

        self.go_to_destination_behaviour.add_effect(
            effect=Effect(
                sensor_name=has_tasks_assigned_sensor.name,
                indicator=-1.0,
                sensor_type=bool))

        # TODO: assemble finished products
        self.assemble_product_behaviour = None


        # Goal is to finish all tasks
        self._job_performance_goal = GoalBase(
            name='job_performance',
            permanent=True,
            plannerPrefix=agent._agent_name,
            conditions=[Negation(self.has_tasks__assigned_condition)])


class GoToStorageBehaviour(GotoLocationBehaviour):

    def __init__(self, agent,plannerPrefix,  **kwargs):
        super(GoToStorageBehaviour, self).__init__(
            agent._agent_name,
            plannerPrefix=plannerPrefix,
            name="go_to_storage",
            graph_name="storage",
            **kwargs)
        self._selected_facility = None
        self.taskKnowledge = TaskKnowledge()
        self.facility_knowledge = FacilityKnowledgebase()

    def _select_pos(self):
        task = self.taskKnowledge.get_task(agent_name=self._agent_name, status="assigned")
        facility = self.facility_knowledge.storages[task.destination]
        return facility.pos
