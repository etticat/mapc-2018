import random

import rospy
from diagnostic_msgs.msg import KeyValue
from mac_ros_bridge.msg import GenericAction

from agent_knowledge.assist import AssistKnowledgebase
from agent_knowledge.movement import MovementKnowledge
from agent_knowledge.resource import ResourceKnowledgebase
from agent_knowledge.tasks import TaskKnowledge
from behaviour_components.behaviours import BehaviourBase
from behaviours.generic_action import GenericActionBehaviour, Action
from behaviours.movement import GotoLocationBehaviour, GoToFacilityBehaviour
from common_utils.agent_utils import AgentUtils
from common_utils.facility_provider import FacilityProvider
from common_utils.product_provider import ProductProvider


class GoToResourceBehaviour(GotoLocationBehaviour):
    def __init__(self, agent, plannerPrefix, **kwargs):
        super(GoToResourceBehaviour, self).__init__(
            agent._agent_name,
            plannerPrefix=plannerPrefix,
            **kwargs)
        self._selected_facility = None
        self.taskKnowledge = TaskKnowledge()
        self._product_provider = ProductProvider(agent_name=agent._agent_name)
        self.facility_knowledge = ResourceKnowledgebase()

    def _select_pos(self):
        neededIngredients = self._product_provider.get_required_ingredients(self._agent_name)
        if len(neededIngredients) > 0:
            # TODO: Select the next ingredient to gather somehow smarter. Like going to the closest one
            for neededIngredient in neededIngredients:
                facilities = self.facility_knowledge.get_resources_for_item(neededIngredient)
                if len(facilities) > 0:
                    # TODO: Select this smarter. Maybe the closest one
                    self._selected_destination = facilities[0].name
                    return facilities[0].pos

        else:
            rospy.logerr("%s:: Trying to go to resource but there is no ingredient left to gather.", self.name)

        rospy.logerr("%s:: Can't find resource node for any required ingredient. %s", self.name, str(neededIngredients))
        self._selected_destination = "none"
        return None


class GoToStorageBehaviour(GotoLocationBehaviour):

    def __init__(self, agent, plannerPrefix, **kwargs):
        super(GoToStorageBehaviour, self).__init__(
            agent._agent_name,
            plannerPrefix=plannerPrefix,
            **kwargs)
        self._selected_facility = None
        self.taskKnowledge = TaskKnowledge()
        self.facility_provider = FacilityProvider()

    def start(self):
        super(GoToStorageBehaviour, self).start()

    def _select_pos(self):
        task = self.taskKnowledge.get_tasks(agent_name=self._agent_name, status="assigned")
        # Goint to any destination
        # TODO: Check this. The goal is to only take tasks with the same destination, so this should be fine.
        # Maybe this can be done more elegant
        if len(task) > 0:
            return self.facility_provider.get_storage_by_name(task[0].destination).pos


class GatherBehaviour(GenericActionBehaviour):

    def __init__(self, name, agent_name, behaviour_name, **kwargs):
        super(GatherBehaviour, self) \
            .__init__(name=name,
                      agent_name=agent_name,
                      action_type=Action.GATHER,
                      **kwargs)
        self._movement_knowledge = MovementKnowledge()
        self._task_knowledge = TaskKnowledge()
        self._product_provider = ProductProvider(agent_name=agent_name)
        self.movement_behaviour_name = behaviour_name
        self.agent_name = agent_name

    def do_step(self):
        required_ingredients = self._product_provider.get_required_ingredients(self._agent_name)
        movement = self._movement_knowledge.get_movement(
            agent_name=self.agent_name,
            behaviour_name=self.movement_behaviour_name
        )

        # If we don't need item anymore stop gathering
        # TODO: This could be moved to a seperate sensor
        if movement.destination not in required_ingredients:
            self._movement_knowledge.stop_movement(self.agent_name, self.movement_behaviour_name)

        super(GatherBehaviour, self).do_step()


class AssembleProductBehaviour(BehaviourBase):

    def __init__(self, agent_name, **kwargs):
        super(AssembleProductBehaviour, self) \
            .__init__(
            requires_execution_steps=True,
            **kwargs)
        self._agent_name = agent_name
        self._task_knowledge = TaskKnowledge()
        self._product_provider = ProductProvider(agent_name=agent_name)
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
        products = self._product_provider.get_required_finished_products(self._agent_name)
        if len(products) > 0:
            rospy.loginfo("AssembleProductBehaviour:: assembling %s", products[0])
            self.action_assemble(products[0])

    def stop(self):
        # Once assembly is done, free all agents from assignemt task
        self._assist_knowledge.cancel_assist_requests(agent_name=self._agent_name)

        super(AssembleProductBehaviour, self).stop()


class GoToWorkshopBehaviour(GoToFacilityBehaviour):

    def __init__(self, **kwargs):
        super(GoToWorkshopBehaviour, self).__init__(
            **kwargs)
        self._selected_facility = None
        self._task_knowledge = TaskKnowledge()
        self._product_provider = ProductProvider(self.agent._agent_name)
        self.assist_knowledge = AssistKnowledgebase()

    def do_step(self):

        super(GoToWorkshopBehaviour, self).do_step()
        if self._selected_pos != None:
            # TODO: only do this once
            self.request_assist()

    def request_assist(self):
        # TODO: This should be done in own behaviour ideally
        finished_products = self._product_provider.get_required_finished_products(self._agent_name)
        for product in finished_products:
            # TODO: Do this recursive so we can make items that are needed to build other items
            for role in self._product_provider.get_product_by_name(product).required_roles:
                self.assist_knowledge.request_assist(agent_name=self._agent_name, role=role, pos=self._selected_pos)
        pass


class DeliverJobBehaviour(BehaviourBase):

    def __init__(self, agent_name, **kwargs):
        super(DeliverJobBehaviour, self) \
            .__init__(
            requires_execution_steps=True,
            **kwargs)
        self._agent_name = agent_name
        self._task_knowledge = TaskKnowledge()
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




class GoToResourceForHoardingBehaviour(GotoLocationBehaviour):
    def __init__(self, agent, plannerPrefix, **kwargs):
        super(GoToResourceForHoardingBehaviour, self).__init__(
            agent._agent_name,
            plannerPrefix=plannerPrefix,
            **kwargs)
        self._selected_facility = None
        self._task_knowledge = TaskKnowledge()
        self._resource_knowledge = ResourceKnowledgebase()

    def _select_pos(self):
        facility = self._resource_knowledge.get_resources_for_item("*")
        if len(facility) > 0:
            facility = random.choice(facility)
            self._selected_destination = facility.name
            return facility.pos
        return None



class GatherForHoardingBehaviour(GenericActionBehaviour):

    def __init__(self, name, agent_name, **kwargs):
        super(GatherForHoardingBehaviour, self) \
            .__init__(name=name,
                      agent_name=agent_name,
                      action_type=Action.GATHER,
                      **kwargs)
