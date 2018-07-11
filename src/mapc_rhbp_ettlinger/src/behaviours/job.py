from mapc_rhbp_ettlinger.msg import Task

import rospy
from diagnostic_msgs.msg import KeyValue
from mac_ros_bridge.msg import GenericAction, Agent, Item

from behaviour_components.behaviours import BehaviourBase
from common_utils.calc import CalcUtil
from provider.action_provider import Action
from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from provider.action_provider import ActionProvider
from provider.facility_provider import FacilityProvider
from provider.product_provider import ProductProvider
from rhbp_selforga.behaviours import DecisionBehaviour

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.behaviours.job')


class DeliverJobBehaviour(DecisionBehaviour):

    def __init__(self, name, agent_name, mechanism, **kwargs):
        super(DeliverJobBehaviour, self) \
            .__init__(name=name, mechanism=mechanism, requires_execution_steps=True, **kwargs)
        self.product_provider = ProductProvider(agent_name=agent_name)
        self.action_provider = ActionProvider(agent_name=agent_name)
        self.current_task = None
        self.items_to_retrive = None
        self.items_to_retrive_count = 0
        self._agent_name = agent_name
        rospy.Subscriber(AgentUtils.get_bridge_topic_prefix(agent_name=self._agent_name) + "agent", Agent,
                         self._action_request_agent)

    def action_deliver_job(self, job):
        self.action_provider.send_action(action_type=Action.DELIVER_JOB, params=[
            KeyValue("Job", str(job))])

    def _action_request_agent(self, agent):
        """

        :param agent:
        :type agent: Agent
        :return:
        """
        if self.current_task is not None and agent.last_action == "deliver_job":
            if sum(self.current_task.items.values()) == 0:
                ettilog.logerr("DeliverJobBehaviour(%s):: Deleting own task. Status: %s", agent.last_action_result,
                               agent.last_action_result)
                self.mechanism.end_task()
                self.current_task = None
        if self.current_task is not None and agent.last_action in [Action.RETRIEVE, Action.RETRIEVE_DELIVERED]:
            self.current_task.items[self.items_to_retrive] = self.current_task.items.get(self.items_to_retrive,
                                                         self.items_to_retrive_count) - self.items_to_retrive_count

    def stop(self):
        self.current_task = None
        super(DeliverJobBehaviour, self).stop()

    def do_step(self):
        if self.current_task is None:
            self.current_task = super(DeliverJobBehaviour, self).do_step()
            self.current_task.items = CalcUtil.dict_from_strings(self.current_task.items)

        if sum(self.current_task.items.values()) > 0:
            item_to_pick_up, count = self.get_item_to_pickup()
            if item_to_pick_up is not None:
                self.pick_up_item(item_to_pick_up, count)
                return

        self.items_to_retrive = None
        self.deliver()

    def deliver(self):
        ettilog.loginfo("DeliverJobBehaviour:: delivering for job %s", self.current_task.task)
        self.action_deliver_job(self.current_task.task)

    def pick_up_item(self, item_to_pick_up, count):
        self.facility_provider = FacilityProvider()
        storage = self.facility_provider.get_storage_by_name(self.current_task.destination_name)

        for item in storage.items:
            if item.name == item_to_pick_up:
                if item.stored >= count:
                    self.retrive_action(action_type=Action.RETRIEVE, count=count, item_to_pick_up=item_to_pick_up)
                elif item.delivered >= count:
                    self.retrive_action(action_type=Action.RETRIEVE_DELIVERED, count=count, item_to_pick_up=item_to_pick_up)
                elif item.stored > 0:
                    self.retrive_action(action_type=Action.RETRIEVE, count=count, item_to_pick_up=item.stored)
                elif item.delivered > 0:
                    self.retrive_action(action_type=Action.RETRIEVE_DELIVERED, count=count, item_to_pick_up=item.delivered)
                else:
                    # TODO: Handle this? cancel job?
                    ettilog.logerr("DeliverJobBehaviour(%s):: Not enough items to retrive from storage available",
                                   self._agent_name)
                return

        # TODO: Handle this? cancel job?
        ettilog.logerr("DeliverJobBehaviour(%s):: Items to retrive from storage not available", self._agent_name)

    def retrive_action(self, action_type, count, item_to_pick_up):
        self.items_to_retrive = item_to_pick_up
        self.items_to_retrive_count = count
        self.action_provider.send_action(action_type=action_type,
                                         params=[KeyValue(key="Item", value=str(item_to_pick_up)),
                                                 KeyValue(key="Amount", value=str(count))])

    def get_item_to_pickup(self):
        item_to_pick_up = (None, 0)
        # Try to pick up all items of a certain type
        for item, count in self.current_task.items.iteritems():
            total_item_volume = self.product_provider.get_volume_of_item(item) * count
            if total_item_volume <= self.product_provider.load_free:
                item_to_pick_up = (item, count)
                break
        # If we can't pick up all items at once we try to pick them up partly
        if item_to_pick_up is None:
            for item, count in self.current_task.items.iteritems():
                volume = self.product_provider.get_volume_of_item(item)
                items_to_retrive = self.product_provider.load_free / volume
                item_to_pick_up = (item, count)
                break
        return item_to_pick_up
