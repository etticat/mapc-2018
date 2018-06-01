import rospy
from diagnostic_msgs.msg import KeyValue
from mac_ros_bridge.msg import Position, GenericAction

from agent_common.agent_utils import AgentUtils
from agent_common.job_utils import JobUtils
from agent_knowledge.assist import AssistKnowledgebase
from agent_knowledge.facilities import FacilityKnowledgebase
from agent_knowledge.movement import MovementKnowledge
from agent_knowledge.tasks import TaskKnowledge
from behaviour_components.activators import BooleanActivator, ThresholdActivator
from behaviour_components.behaviours import BehaviourBase
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition, Negation
from behaviour_components.goals import GoalBase
from behaviour_components.network_behavior import NetworkBehaviour
from behaviours.generic_action import GenericActionBehaviour, Action
from behaviours.movement import GotoLocationBehaviour, GoToFacilityBehaviour
from rhbp_utils.knowledge_sensors import KnowledgeSensor
from sensor.job import IngredientSensor, FinishedProductSensor, AmountInListActivator
from sensor.movement import DestinationDistanceSensor


class GoToWorkshopBehaviour(GoToFacilityBehaviour):

    def __init__(self, **kwargs):
        super(GoToWorkshopBehaviour, self).__init__(
            **kwargs)
        self._selected_facility = None
        self.taskKnowledge = TaskKnowledge(self.agent._agent_name)
        self.facility_knowledge = FacilityKnowledgebase()
        self.assist_knowledge = AssistKnowledgebase()


    def do_step(self):

        super(GoToWorkshopBehaviour, self).do_step()
        if self._selected_pos != None:
            # TODO: only do this once
            self.request_assist()

    def request_assist(self):
        # TODO: This should be done in own behaviour ideally
        finished_products = self.taskKnowledge.get_required_finished_products(self._agent_name)
        for product in finished_products:
            # TODO: Do this recursive so we can make items that are needed to build other items
            for role in self.taskKnowledge.products[product].required_roles:
                self.assist_knowledge.request_assist(agent_name=self._agent_name, role = role, pos=self._selected_pos)
        pass


class DeliverJobBehaviour(BehaviourBase):

    def __init__(self, agent_name, **kwargs):
        super(DeliverJobBehaviour, self) \
            .__init__(
            requires_execution_steps=True,
            **kwargs)
        self._agent_name = agent_name
        self._task_knowledge = TaskKnowledge(agent_name=agent_name)
        self._pub_generic_action = rospy.Publisher(
            name=AgentUtils.get_bridge_topic_prefix(agent_name) + 'generic_action',
            data_class=GenericAction,
            queue_size=10)

    def action_deliver_job(self, job):

        action = GenericAction()
        action.action_type = Action.DELIVER_JOB
        action.params = [
            KeyValue("Job", str(job))]

        self._pub_generic_action.publish(action)

    def do_step(self):
        tasks = self._task_knowledge.get_tasks(self._agent_name, status="assigned")

        if len(tasks) > 0:
            rospy.loginfo("DeliverJobBehaviour:: delivering for job %s", tasks[0].job_id)
            self.action_deliver_job(tasks[0].job_id)



class JobPerformanceNetwork(NetworkBehaviour):

    def __init__(self, agent, name, msg, **kwargs):

        proximity = msg.proximity

        super(JobPerformanceNetwork, self).__init__(name, **kwargs)

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
        rospy.logerr("44")

    def init_gather_behaviour(self, agent):
        self.gather_ingredients_behaviour = GatherBehaviour(
            name="gather_behaviour",
            agent_name=agent._agent_name,
            plannerPrefix=self.get_manager_prefix())

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
            plannerPrefix=self.get_manager_prefix(),
            ingredientSensor=self.has_all_ingredients_sensor)

        self.go_to_resource_node_behaviour.add_precondition(
            precondition=Negation(self.has_all_ingredients_condition)
        )

        self.storage_destination_sensor = DestinationDistanceSensor(
            name='resource_destination_sensor',
            agent_name=agent._agent_name,
            behaviour_name=self.go_to_resource_node_behaviour._name)

        self.at_resource_node_condition = Condition(
            sensor=self.storage_destination_sensor,
            activator=ThresholdActivator(
                thresholdValue=proximity,
                isMinimum=False))  # highest activation if the value is below threshold


        self.go_to_resource_node_behaviour.add_precondition(
            precondition=Negation(self.at_resource_node_condition))

        self.go_to_resource_node_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.storage_destination_sensor.name,
                indicator=-1.0,
                sensor_type=float

            )
        )

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
            pattern=TaskKnowledge.get_tuple_task_creation(
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
            **kwargs)
        self._selected_facility = None
        self.taskKnowledge = TaskKnowledge(agent._agent_name)
        self.facility_knowledge = FacilityKnowledgebase()

    def start(self):
        super(GoToStorageBehaviour, self).start()

    def _select_pos(self):
        task = self.taskKnowledge.get_tasks(agent_name=self._agent_name, status="assigned")
        # Goint to any destination
        # TODO: Check this. The goal is to only take tasks with the same destination, so this should be fine.
        # Maybe this can be done more elegant
        if len(task) > 0:
            return self.facility_knowledge.storages[task[0].destination].pos


class GatherBehaviour(GenericActionBehaviour):

    def __init__(self, name, agent_name, **kwargs):
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



class AssembleProductBehaviour(BehaviourBase):

    def __init__(self, agent_name, **kwargs):
        super(AssembleProductBehaviour, self) \
            .__init__(
            requires_execution_steps=True,
            **kwargs)
        self._agent_name = agent_name
        self._task_knowledge = TaskKnowledge(agent_name=agent_name)
        self._assist_knowledge = AssistKnowledgebase()
        self._pub_generic_action = rospy.Publisher(
            name=AgentUtils.get_bridge_topic_prefix(agent_name) + 'generic_action',
            data_class=GenericAction,
            queue_size=10)

    def action_assemble(self, item):
        """
        Specific "goto" action publishing helper function
        :param facility_name: name of the facility we want to go to
        :param publisher: publisher to use
        """
        action = GenericAction()
        action.action_type = Action.ASSEMBLE
        action.params = [
            KeyValue("item", str(item))]
        
        self._pub_generic_action.publish(action)

    def do_step(self):
        products = self._task_knowledge.get_required_finished_products(self._agent_name)
        if len(products) > 0:
            rospy.loginfo("AssembleProductBehaviour:: assembling %s", products[0])
            self.action_assemble(products[0])

    def stop(self):
        # Once assembly is done, free all agents from assignemt task
        self._assist_knowledge.cancel_assist_requests(agent_name=self._agent_name)

        super(AssembleProductBehaviour, self).stop()