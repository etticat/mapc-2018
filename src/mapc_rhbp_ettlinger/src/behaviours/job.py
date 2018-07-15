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
    """
    Behaviour that allows delivering of items to a storage for a specific job
    """

    def __init__(self, name, agent_name, mechanism, **kwargs):
        super(DeliverJobBehaviour, self) \
            .__init__(name=name, mechanism=mechanism, requires_execution_steps=True, **kwargs)

        self._agent_name = agent_name

        # Initialise providers
        self._product_provider = ProductProvider(agent_name=agent_name)
        self._action_provider = ActionProvider(agent_name=agent_name)
        self.facility_provider = FacilityProvider()

        self._current_task = None
        self._items_to_retrieve = None
        self._items_to_retrieve_count = 0

        rospy.Subscriber(AgentUtils.get_bridge_topic_prefix(agent_name=self._agent_name) + "agent", Agent,
                         self._action_request_agent)

    def _action_request_agent(self, agent):
        """
        Check if delivery_job task is done to save it in the mechanism
        Also checks the status of the retrieve action
        :param agent:
        :type agent: Agent
        :return:
        """
        if self._current_task is not None :
            if agent.last_action == Action.DELIVER_JOB:
                # If there are no more items to be delivered, end task but keep job tasks of other agents active, so
                # they can finish it
                if sum(self.items_from_storage.values()) == 0:
                    ettilog.loginfo("DeliverJobBehaviour(%s):: Finished job delivery. Status: %s", self._agent_name, agent.last_action_result)
                    self.mechanism.end_task(notify_others =False)
                    self._current_task = None

                # Cancel the job delivery if something went wrong.
                if agent.last_action_result not in ["successful_partial", "successful"]:
                    ettilog.logerr("DeliverJobBehaviour(%s):: Job delivery failed. cancelling all. Status: %s", self._agent_name, agent.last_action_result)
                    self.mechanism.end_task(notify_others =True)
                    self._current_task = None

            if agent.last_action in [Action.RETRIEVE, Action.RETRIEVE_DELIVERED] :
                # after successfully retreiving items, update the storage dict
                if agent.last_action_result is "successful":
                    ettilog.loginfo("DeliverJobBehaviour(%s):: Retreived %dx%s status:", self._agent_name, self._items_to_retrieve_count, self._items_to_retrieve, agent.last_action_result)
                    self.items_from_storage[self._items_to_retrieve] = self.items_from_storage.get(self._items_to_retrieve,
                                                                                                   self._items_to_retrieve_count) - self._items_to_retrieve_count
                    # If something went wrong during retreival, cancel job
                    if agent.last_action_result not in ["successful_partial", "successful"]:
                        ettilog.logerr("DeliverJobBehaviour(%s):: Item retrieval failed. cancelling all. Status: %s", self._agent_name, agent.last_action_result)
                        self.mechanism.end_task(notify_others=True)
                        self._current_task = None

    def stop(self):
        """
        remove the current item when the behaviour is stopped, to make sure the action_request does not catch expired
        callbacks
        :return:
        """
        self._current_task = None
        super(DeliverJobBehaviour, self).stop()

    def do_step(self):
        """
        Performing job_deliver as well as retrieval when necessary
        :return:
        """

        # fetch task from mechanism
        if self._current_task is None:
            self._current_task = super(DeliverJobBehaviour, self).do_step()
            self._current_task.items = CalcUtil.dict_from_string_list(self._current_task.items)
            ettilog.loginfo("DeliverJobBehaviour(%s):: Starting doing job %s items: %s", self._agent_name, self._current_task.task, self._current_task.items)

        # Calculate items that we still need to pick up from storage
        self.items_from_storage = CalcUtil.dict_diff(self._current_task.items, self._product_provider.get_stock_items(), normalize_to_zero=True)
        ettilog.logerr("DeliverJobBehaviour(%s):: From storage still needed: %s", self._agent_name, self.items_from_storage)

        # Find an item that we still need and if we find one, send the pick up item stock, if not, go on to deliver
        # In the case where we can't pick up anything because the capacity is full, just go on to deliver free up space
        # to pick up items in the next round
        if sum(self.items_from_storage.values()) > 0:
            item_to_pick_up, count = self.get_item_to_pickup()
            if item_to_pick_up is not None:
                self.pick_up_item(item_to_pick_up, count)
                return

        # Update delivery goal with the items that still need to be delivered
        self._product_provider.update_delivery_goal(
            item_list=self._current_task.items,
            job_id=self._current_task.task,
            storage=self._current_task.destination_name)

        # If we reach this code, retrival was not neccessary, therefore reset the value
        self._items_to_retrieve = None

        # start deliver action
        self._action_provider.send_action(action_type=Action.DELIVER_JOB, params=[
            KeyValue("Job", str(self._current_task.task))])

    def pick_up_item(self, item_to_pick_up, item_count):
        """
        Selects the best option to pick up items from a storage
        :param item_to_pick_up: the item, which we need to pick up
        :type item_to_pick_up: str
        :param item_count: Number of items, the agent needs for the job and can also fit in their stock
        :type item_count:
        :return:
        """
        # Get a current instance of the storage object
        storage = self.facility_provider.get_storage_by_name(self._current_task.destination_name)

        # Pick retrieval method and possible number of items to retrieve
        for item in storage.items:
            if item.name == item_to_pick_up:
                items_ready_for_retrieval = min(item_count, item.stored)
                items_ready_for_retrieval_delivery = min(item_count, item.delivered)

                # Retrieve items if we can
                if items_ready_for_retrieval > 0 or items_ready_for_retrieval_delivery > 0:
                    # Prefer the item delivery method, where we can pick up more items at once.
                    # If both methods can provide the same amount, prefer retrieve, as it costs capacity
                    if items_ready_for_retrieval >= items_ready_for_retrieval_delivery:
                        self.retrieve_action(action_type=Action.RETRIEVE, item_count=items_ready_for_retrieval, item_to_pick_up=item_to_pick_up)
                    else:
                        self.retrieve_action(action_type=Action.RETRIEVE_DELIVERED, item_count=item_count, item_to_pick_up=items_ready_for_retrieval_delivery)
                else:
                    # If retrieval was impossible, cancel job with error
                    self.mechanism.end_task(notify_others=True)
                    ettilog.logerr("DeliverJobBehaviour(%s):: Not enough items to retrieve from storage available",
                                   self._agent_name)
                return

        # If retrieval was impossible, cancel job with error
        self.mechanism.end_task(notify_others=True)
        ettilog.logerr("DeliverJobBehaviour(%s):: Items to retrive from storage not available tryig to pick up %s from storage %s", self._agent_name, str(item_to_pick_up), self._current_task.destination_name)

    def retrieve_action(self, action_type, item_count, item_to_pick_up):
        """
        Retrieves Items from the storage and saves the items to retreive for the callback method to use.
        :param action_type: Action type used to retreive the item
        :param item_count: number of items to retreive
        :param item_to_pick_up: item to pick up
        :return:
        """
        ettilog.loginfo("DeliverJobBehaviour(%s):: Going to pick up %dx%s using %s", self._agent_name, item_count, item_to_pick_up, action_type)
        self._items_to_retrieve = item_to_pick_up
        self._items_to_retrieve_count = item_count
        self._action_provider.send_action(action_type=action_type,
                                          params=[KeyValue(key="Item", value=str(item_to_pick_up)),
                                                  KeyValue(key="Amount", value=str(item_count))])

    def get_item_to_pickup(self):
        """
        Finds and returns an item and its count to be picked up
        :return: tuple
        """
        item_to_pick_up = (None, 0)

        # Try to pick up all items of a certain type
        for item, item_count in self.items_from_storage.iteritems():
            # Only pick up items where we need more than 0 -> Dict may have zero values
            if item_count > 0:
                total_item_volume = self._product_provider.get_volume_of_item(item) * item_count
                if total_item_volume <= self._product_provider.load_free:
                    item_to_pick_up = (item, item_count)
                    break

        # If we can't pick up all items at once we try to pick them up partly
        if item_to_pick_up[0] is None:
            for item, item_count in self.items_from_storage.iteritems():
                if item_count > 0:
                    for item, item_count in self.items_from_storage.iteritems():
                        volume = self._product_provider.get_volume_of_item(item)
                        # Get the number of items, that fit in agent stock
                        items_to_retrieve = self._product_provider.load_free / volume
                        item_to_pick_up = (item, items_to_retrieve)
                        break

        return item_to_pick_up
