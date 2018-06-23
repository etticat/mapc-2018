import rospy
from diagnostic_msgs.msg import KeyValue
from mac_ros_bridge.msg import GenericAction, Agent
from mapc_rhbp_ettlinger.msg import AssembleTaskProgress, AssembleStop

from agent_knowledge.resource import ResourceKnowledgebase
from agent_knowledge.task import TaskKnowledgebase
from behaviour_components.behaviours import BehaviourBase
from behaviours.generic_action import GenericActionBehaviour, Action
from behaviours.movement import GotoLocationBehaviour
from common_utils import rhbp_logging
from common_utils.agent_utils import AgentUtils
from provider.facility_provider import FacilityProvider
from provider.product_provider import ProductProvider

ettilog = rhbp_logging.LogManager(logger_name=rhbp_logging.LOGGER_DEFAULT_NAME + '.behaviours.job')

class GoToResourceBehaviour(GotoLocationBehaviour):
    """
    Behaviour that allows agents to move to a resource behaviour, so they can gather items for hoarding or performing jobs
    """
    def __init__(self, agent, plannerPrefix, **kwargs):
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
        self._selected_facility = None
        self._agent = agent
        self._task_knowledge = JobKnowledgebase()
        self._product_provider = ProductProvider(agent_name=agent._agent_name)
        self._facility_knowledge = ResourceKnowledgebase()

    def _select_pos(self):
        neededIngredients = self._product_provider.get_planned_ingredients()
        if len(neededIngredients) > 0:
            facilities = self._facility_knowledge.get_resources_for_items(neededIngredients[0])
            if len(facilities) > 0:
                closest_facility = AgentUtils.calculate_closest_facility(self._agent.agent_info.pos, facilities)
                self._selected_destination = closest_facility.name
                return closest_facility.pos

        else:
            ettilog.logerr("%s:: Trying to go to resource but there is no ingredient left to gather.", self.name)

        ettilog.logerr("%s:: Can't find resource node for any required ingredient. %s", self.name, str(neededIngredients))
        self._selected_destination = "none"
        return None


class GoToStorageForJobBehaviour(GotoLocationBehaviour):
    """
    Behaviour, that allows going to the Storage location to finish a job
    """
    def __init__(self, agent, plannerPrefix, **kwargs):
        super(GoToStorageForJobBehaviour, self).__init__(
            agent_name=agent._agent_name,
            plannerPrefix=plannerPrefix,
            **kwargs)
        self._selected_facility = None
        self.taskKnowledge = JobKnowledgebase()
        self.facility_provider = FacilityProvider()

    def start(self):
        super(GoToStorageForJobBehaviour, self).start()

    def _select_pos(self):
        task = self.taskKnowledge.get_task(agent_name=self._agent_name)
        if task is not None:
            return task.pos

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
        task = self.taskKnowledge.get_task(agent_name=self._agent_name)
        if task is not None:
            return task.pos


class GatherBehaviour(GenericActionBehaviour):
    """
    The behaviour allows gathering items for further assembly at a resource node
    """
    def __init__(self, name, agent_name, behaviour_name, **kwargs):
        super(GatherBehaviour, self) \
            .__init__(name=name,
                      agent_name=agent_name,
                      action_type=Action.GATHER,
                      **kwargs)
        self._movement_knowledge = TaskKnowledgebase()
        self._task_knowledge = JobKnowledgebase()
        self._product_provider = ProductProvider(agent_name=agent_name)
        self._resource_knowledgebase= ResourceKnowledgebase()
        self.movement_behaviour_name = behaviour_name
        self.agent_name = agent_name

    def do_step(self):
        required_ingredients = self._product_provider.get_planned_ingredients()
        movement = self._movement_knowledge.get_task(
            agent_name=self.agent_name,
            type=TaskKnowledgebase.TYPE_GATHERING
        )

        # If we don't need item anymore stop gathering
        resource = self._resource_knowledgebase.get_resource_by_name(movement.destination)
        if resource != None and resource.item.name not in required_ingredients:
            self._movement_knowledge.finish_task(self.agent_name, self.movement_behaviour_name)

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
        self._task = None
        self._agent_name = agent_name
        self._last_task = None
        self._last_goal = None
        self._product_provider = ProductProvider(agent_name=agent_name)
        self._movement_knowledge = TaskKnowledgebase()
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
            ettilog.logerr("AssembleProductBehaviour(%s):: Last assembly: %s", self._agent_name, agent.last_action_result)
            if agent.last_action_result in ["successful", "failed_capacity"]:
                self._task_progress_dict[self._task.id] = self._task_progress_dict.get(self._task.id, 0) + 1
                if self._get_assemble_step() < len(self._task.task.split(",")):
                    # If there are still tasks to do, inform all others that the next task will be performed
                    ettilog.logerr("AssembleProductBehaviour(%s):: Finished assembly of product, going on to next task ....", self._agent_name)
                    assembleTaskCoordination = AssembleTaskProgress(
                        id=self._task.id,
                        step=self._get_assemble_step()
                    )
                    self._pub_assemble_progress.publish(assembleTaskCoordination)
                else:
                    # If this was the last task -> cancel the assembly
                    ettilog.logerr("AssembleProductBehaviour(%s):: Last product of assembly task assembled, ending assembly", self._agent_name)
                    self._pub_assemble_stop.publish(AssembleStop(id=self._task.id, reason="assembly finished"))

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

        self._task = self._movement_knowledge.get_task(agent_name=self._agent_name, type=TaskKnowledgebase.TYPE_ASSEMBLE)

        super(AssembleProductBehaviour, self).start()

    def do_step(self):
        assert self._task != None
        products = self._task.task.split(",")
        if len(products) > 0 and self._get_assemble_step() < len(products):
            (self._last_task, self._last_goal) = products[self._get_assemble_step()].split(":")
            if self._last_task == "assemble":
                ettilog.logerr("AssembleProductBehaviour(%s):: step %d/%d current task: %s", self._agent_name,
                             self._get_assemble_step() + 1, len(products), products[self._get_assemble_step()])
                self.action_assemble(self._last_goal)
            elif self._last_task == "assist":
                self.action_assist_assemble(self._last_goal)
            else:
                ettilog.logerr("AssembleProductBehaviour(%s):: Invalid task", self._agent_name)

        else:
            ettilog.logerr("This should never happen. Assembly is executed after all tasks are finished")

    def stop(self):
        # Once assembly is done, free all agents from assignemt task

        self._task = None
        super(AssembleProductBehaviour, self).stop()

    def _get_assemble_step(self):
        return self._task_progress_dict.get(self._task.id, 0)

class DeliverJobBehaviour(BehaviourBase):

    def __init__(self, agent_name, **kwargs):
        super(DeliverJobBehaviour, self) \
            .__init__(
            requires_execution_steps=True,
            **kwargs)
        self.current_task = None
        self._agent_name = agent_name
        self._task_knowledge = TaskKnowledgebase()
        self._pub_generic_action = rospy.Publisher(
            name=AgentUtils.get_bridge_topic_prefix(agent_name) + 'generic_action',
            data_class=GenericAction,
            queue_size=10)
        rospy.Subscriber(AgentUtils.get_bridge_topic_prefix(agent_name=self._agent_name) + "agent", Agent, self._action_request_agent)


    def action_deliver_job(self, job):
        action = GenericAction()
        action.action_type = Action.DELIVER_JOB
        action.params = [
            KeyValue("Job", str(job))]

        self._pub_generic_action.publish(action)

    def action_build_well(self, type):
        action = GenericAction()
        action.action_type = Action.BUILD
        action.params = [
            KeyValue("WellType", str(type))]

        self._pub_generic_action.publish(action)
    def _action_request_agent(self, agent):
        """

        :param agent:
        :type agent: Agent
        :return:
        """
        # TODO: Also add a timeout here: if it doesnt work for 5 steps -> Fail with detailed error
        if self.current_task != None and agent.last_action == "deliver_job":
            ettilog.logerr("DeliverJobBehaviour(%s):: Deleting own task. Status: %s", agent.last_action_result, agent.last_action_result)
            self._task_knowledge.finish_task(agent_name=self._agent_name, type=TaskKnowledgebase.TYPE_DELIVER)
            self.current_task = None


    def stop(self):
        self.current_task = None
        super(DeliverJobBehaviour, self).stop()

    def do_step(self):
        if self.current_task == None:
            self.current_task = self._task_knowledge.get_task(self._agent_name)

        if self.current_task != None:
            ettilog.loginfo("DeliverJobBehaviour:: delivering for job %s", self.current_task.job_id)
            self.action_deliver_job(self.current_task.job_id)
            # TODO: Check what happens when the delivery fails ->
            # TODO: Pass an acnowledgement object into the mac ros bridge. Once it finishes/fails it notifies the application


