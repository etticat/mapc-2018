from diagnostic_msgs.msg import KeyValue

import rospy
from mac_ros_bridge.msg import GenericAction, Position

from common_utils.agent_utils import AgentUtils
from common_utils.singleton import Singleton


class Action(object):
    DISMANTLE = "dismantle"
    BUILD = "build"
    GO_TO = "goto"
    RECEIVE = "receive"
    GIVE = "give"
    STORE = "store"
    RETRIEVE = "retrieve"
    ASSEMBLE = "assemble"
    ASSIST_ASSEMBLE = "assist_assemble"
    BUY = "buy"
    RESELL = "resell"
    RESELL_ALL = "resell_all"
    DELIVER_JOB = "deliver_job"
    RETRIEVE_DELIVERED = "retrieve_delivered"
    BID_FOR_JOB = "bid_for_job"
    DUMP = "dump"
    CHARGE = "charge"
    CONTINUE = "continue"
    ABORT = "abort"
    SKIP = "skip"
    POST_JOB = "post_job"
    GATHER = "gather"
    RECHARGE = "recharge"
    REPAIR = "repair"


class ActionProvider(object):
    """
    Provider for communication with the mac_ros bridge.
    It allows sending actions that are then performed on massim simulation
    When an action is sent, it sets a flag indicating this, so the planner can be stopped.
    """
    __metaclass__ = Singleton

    def __init__(self, agent_name):
        self._pub_generic_action = rospy.Publisher(
            name=AgentUtils.get_bridge_topic_prefix(agent_name) + 'generic_action',
            data_class=GenericAction,
            queue_size=10)

        self._action_response_found = False

    def send_action(self, action_type, params=None):
        """
        Sends an action to the mac_ros_bridge
        :param action_type:
        :param params:
        :return:
        """
        if params is None:
            params = []

        action = GenericAction()
        action.action_type = action_type
        action.params = params
        self._pub_generic_action.publish(action)
        self._action_response_found = True

    def action_go_to_position(self, pos):
        """
        Sends the go_to action to the mac ros bridge using Position
        :param pos:
        :type pos: Position
        :return:
        """
        self.send_action(action_type=Action.GO_TO, params=[
            KeyValue("latitude", str(pos.lat)),
            KeyValue("longitude", str(pos.long))
        ])

    def action_go_to_destination(self, destination):
        """
        Sends the go_to action to the mac ros bridge using either a Position or an object with attribute pos : Position
        :param destination:
        :type destination: Position or Facility or Task
        :return:
        """
        if isinstance(destination, Position):
            self.action_go_to_position(pos=destination)
        else:
            self.action_go_to_position(pos=destination.pos)

    @property
    def action_response_found(self):
        return self._action_response_found

    def reset_action_response_found(self):
        self._action_response_found = False
