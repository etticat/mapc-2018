from mac_ros_bridge.msg import Position

from agent_knowledge.tasks import TaskKnowledge
from behaviour_components.activators import BooleanActivator
from behaviour_components.conditions import Condition
from behaviour_components.goals import GoalBase
from behaviours.movement import GotoLocationBehaviour
from rhbp_utils.knowledge_sensors import KnowledgeSensor


class JobPerformanceGraph:

    def __init__(self, agent_name):

        has_tasks_assigned_sensor = KnowledgeSensor(
            name='has_task',
            pattern=TaskKnowledge.get_tuple_task_creation(
                job_id="*",
                task_id="*",
                destination="*",
                agent=agent_name,
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
        # self.go_to_destination_behaviour = GoToStorageBehaviour(agent_name=agent_name)

        # TODO: assemble finished products
        self.assemble_product_behaviour = None


        # Exploration goal
        self._job_performance_goal = GoalBase(
            name='job_performance',
            permanent=True,
            plannerPrefix=agent_name,
            conditions=[self.has_tasks__assigned_condition])

    def add_precondition(self, precondition):
        """ TODO: add precondition to all behaviours of this graph """
        pass


class GoToStorageBehaviour(GotoLocationBehaviour):

    def _select_pos(self):
        # TODO: Select proper location. For now go somewhere random
        return Position(long=2.32883, lat= 48.88682)
