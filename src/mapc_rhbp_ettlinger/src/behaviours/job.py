import random

import rospy
from diagnostic_msgs.msg import KeyValue
from mac_ros_bridge.msg import GenericAction, Agent
from mapc_rhbp_ettlinger.msg import AssembleTaskProgress, AssembleStop

from agent_knowledge.assemble_task import AssembleKnowledgebase
from agent_knowledge.movement import MovementKnowledgebase
from agent_knowledge.resource import ResourceKnowledgebase
from agent_knowledge.tasks import JobKnowledgebase
from behaviour_components.behaviours import BehaviourBase
from behaviours.generic_action import GenericActionBehaviour, Action
from behaviours.movement import GotoLocationBehaviour, GoToFacilityBehaviour
from common_utils.agent_utils import AgentUtils
from provider.facility_provider import FacilityProvider
from provider.product_provider import ProductProvider


class GoToResourceBehaviour(GotoLocationBehaviour):
    """
    Behaviour that allows agents to move to a resource behaviour, so they can gather items for hoarding or performing jobs
    """
    def __init__(self, agent, plannerPrefix, product_provider_method, **kwargs):
        """

        :param agent:
        :param plannerPrefix:
        :param product_provider_method: This function provides the items that the agent currently needs. TODO: Maybe use a more elegant design pattern for this
        :param kwargs:
        """
        super(GoToResourceBehaviour, self).__init__(
            agent._agent_name,
            plannerPrefix=plannerPrefix,
            **kwargs)
        self._product_provider_method = product_provider_method
        self._selected_facility = None
        self._agent = agent
        self._task_knowledge = JobKnowledgebase()
        self._product_provider = ProductProvider(agent_name=agent._agent_name)
        self._facility_knowledge = ResourceKnowledgebase()

    def _select_pos(self):
        neededIngredients = self._product_provider_method()
        if len(neededIngredients) > 0:
            for neededIngredient in neededIngredients:
                facilities = self._facility_knowledge.get_resources_for_items(neededIngredient)
                if len(facilities) > 0:
                    closest_facility = AgentUtils.calculate_closest_facility(self._agent.agent_info.pos, facilities)
                    self._selected_destination = closest_facility.name
                    return closest_facility.pos

        else:
            rospy.logerr("%s:: Trying to go to resource but there is no ingredient left to gather.", self.name)

        rospy.logerr("%s:: Can't find resource node for any required ingredient. %s", self.name, str(neededIngredients))
        self._selected_destination = "none"
        return None


class GoToStorageBehaviour(GotoLocationBehaviour):
    """
    Behaviour, that allows going to the Storage location to finish a job
    """
    def __init__(self, agent, plannerPrefix, **kwargs):
        super(GoToStorageBehaviour, self).__init__(
            agent_name=agent._agent_name,
            plannerPrefix=plannerPrefix,
            **kwargs)
        self._selected_facility = None
        self.taskKnowledge = JobKnowledgebase()
        self.facility_provider = FacilityProvider()

    def start(self):
        super(GoToStorageBehaviour, self).start()

    def _select_pos(self):
        task = self.taskKnowledge.get_tasks(agent_name=self._agent_name, status="assigned")
        # TODO Maybe this can be done more elegant
        if len(task) > 0:
            return self.facility_provider.get_storage_by_name(task[0].destination).pos


class GatherBehaviour(GenericActionBehaviour):
    """
    The behaviour allows gathering items for further assembly at a resource node
    """
    def __init__(self, name, agent_name, behaviour_name, product_provider_method, **kwargs):
        super(GatherBehaviour, self) \
            .__init__(name=name,
                      agent_name=agent_name,
                      action_type=Action.GATHER,
                      **kwargs)
        self._movement_knowledge = MovementKnowledgebase()
        self._task_knowledge = JobKnowledgebase()
        self._product_provider_method = product_provider_method
        self._resource_knowledgebase= ResourceKnowledgebase()
        self.movement_behaviour_name = behaviour_name
        self.agent_name = agent_name

    def do_step(self):
        required_ingredients = self._product_provider_method()
        movement = self._movement_knowledge.get_movement(
            agent_name=self.agent_name,
            behaviour_name=self.movement_behaviour_name
        )

        # If we don't need item anymore stop gathering
        # TODO: This could be moved to a seperate sensor/behaviour?
        resource = self._resource_knowledgebase.get_resource_by_name(movement.destination)
        if resource != None and resource.item.name not in required_ingredients:
            self._movement_knowledge.stop_movement(self.agent_name, self.movement_behaviour_name)

        super(GatherBehaviour, self).do_step()


class AssembleProductBehaviour(BehaviourBase):
    """
    Behaviour for the assembly of a product
    """
    def __init__(self, agent_name, **kwargs):
        super(AssembleProductBehaviour, self) \
            .__init__(
            requires_execution_steps=True,
            **kwargs)
        self.assemble_task = None
        self._agent_name = agent_name
        self._last_task = None
        self._last_goal = None
        self._task_knowledge = JobKnowledgebase()
        self._product_provider = ProductProvider(agent_name=agent_name)
        self._assemble_knowledgebase = AssembleKnowledgebase()
        self._pub_generic_action = rospy.Publisher(
            name=AgentUtils.get_bridge_topic_prefix(agent_name) + 'generic_action',
            data_class=GenericAction,
            queue_size=10)

        self._task_progress_dict = {}

        self._pub_assemble_progress = rospy.Publisher(
            AgentUtils.get_assemble_prefix() + "progress",
            AssembleTaskProgress,
            queue_size=10)

        rospy.Subscriber(AgentUtils.get_assemble_prefix() + "progress", AssembleTaskProgress, self._callback_task_progress)

        rospy.Subscriber(AgentUtils.get_bridge_topic_prefix(agent_name=self._agent_name) + "agent", Agent, self._action_request_agent)


        self._pub_assemble_stop = rospy.Publisher(AgentUtils.get_assemble_prefix() + "stop", AssembleStop,
                                                        queue_size=10)


    def _callback_task_progress(self, assembleTaskProgress):
        """

        :param assembleTaskProgress:
        :type assembleTaskProgress: AssembleTaskProgress
        :return:
        """
        self._task_progress_dict[assembleTaskProgress.id] = assembleTaskProgress.step

    def _action_request_agent(self, agent):
        """

        :param agent:
        :type agent: Agent
        :return:
        """
        # TODO: Also add a timeout here: if it doesnt work for 5 steps -> Fail with detailed error
        if self._last_task == "assemble" and agent.last_action == "assemble":
            rospy.logerr("AssembleProductBehaviour(%s):: Last assembly: %s", self._agent_name, agent.last_action_result)
            for agent.last_action_result in ["successful", "failed_capacity"]:
                self._task_progress_dict[self.assemble_task.id] = self._task_progress_dict.get(self.assemble_task.id, 0) + 1
                if self._get_assemble_step() < len(self.assemble_task.tasks.split(",")):
                    # If there are still tasks to do, inform all others that the next task will be performed
                    rospy.logerr("AssembleProductBehaviour(%s):: Finished assembly of product, going on to next task ....", self._agent_name)
                    assembleTaskCoordination = AssembleTaskProgress(
                        id=self.assemble_task.id,
                        step=self._get_assemble_step()
                    )
                    self._pub_assemble_progress.publish(assembleTaskCoordination)
                else:
                    # If this was the last task -> cancel the assembly
                    rospy.logerr("AssembleProductBehaviour(%s):: Last product of assembly task assembled, ending assembly", self._agent_name)
                    self._pub_assemble_stop.publish(AssembleStop(id=self.assemble_task.id, reason="assembly finished"))

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

    def action_assist_assemble(self, agent):
        """
        Performs the assist asemble action and publishes it towards the mac_ros_bridge
        :param agent: str
        :return:
        """
        action = GenericAction()
        action.action_type = Action.ASSIST_ASSEMBLE
        action.params = [
            KeyValue("Agent", str(agent))]

        self._pub_generic_action.publish(action)

    def start(self):

        self.assemble_task = self._assemble_knowledgebase.get_assemble_task(agent=self._agent_name)

        super(AssembleProductBehaviour, self).start()

    def do_step(self):
        assert self.assemble_task != None
        products = self.assemble_task.tasks.split(",")
        if len(products) > 0 and self._get_assemble_step() < len(products):
            (self._last_task, self._last_goal) = products[self._get_assemble_step()].split(":")
            if self._last_task == "assemble":
                rospy.logerr("AssembleProductBehaviour(%s):: step %d/%d current task: %s", self._agent_name,
                             self._get_assemble_step() + 1, len(products), products[self._get_assemble_step()])
                self.action_assemble(self._last_goal)
            elif self._last_task == "assist":
                self.action_assist_assemble(self._last_goal)
            else:
                rospy.logerr("AssembleProductBehaviour(%s):: Invalid task", self._agent_name)

        else:
            rospy.logerr("This should never happen. Assembly is executed after all tasks are finished")

    def stop(self):
        # Once assembly is done, free all agents from assignemt task

        self.assemble_task = None
        super(AssembleProductBehaviour, self).stop()

    def _get_assemble_step(self):
        return self._task_progress_dict.get(self.assemble_task.id, 0)


class GoToWorkshopBehaviour(GotoLocationBehaviour):

    def __init__(self, **kwargs):
        super(GoToWorkshopBehaviour, self).__init__(
            **kwargs)
        self._selected_facility = None
        self._task_knowledge = JobKnowledgebase()
        self._product_provider = ProductProvider(agent_name=self._agent_name)
        self._assemble_knowledge = AssembleKnowledgebase()

    def _select_pos(self):
        assemble_task = self._assemble_knowledge.get_assemble_task(agent=self._agent_name)
        if assemble_task is None:
            rospy.logerr("no task assigned")
            return
        return assemble_task.pos

    def start(self):
        self._selected_pos = False
        super(GoToWorkshopBehaviour, self).start()


class DeliverJobBehaviour(BehaviourBase):

    def __init__(self, agent_name, **kwargs):
        super(DeliverJobBehaviour, self) \
            .__init__(
            requires_execution_steps=True,
            **kwargs)
        self._agent_name = agent_name
        self._task_knowledge = JobKnowledgebase()
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
            # TODO: Check what happens when the delivery fails ->
            # TODO: Pass an acnowledgement object into the mac ros bridge. Once it finishes/fails it notifies the application


