from diagnostic_msgs.msg import KeyValue

import rospy
from mac_ros_bridge.msg import GenericAction, Position

from common_utils.agent_utils import AgentUtils
from common_utils.singleton import Singleton


class Action(object):
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
    __metaclass__ = Singleton

    def __init__(self, agent_name):
        self._pub_generic_action = rospy.Publisher(
            name=AgentUtils.get_bridge_topic_prefix(agent_name) + 'generic_action',
            data_class=GenericAction,
            queue_size=10)

        self.current_action_sent = False

    def send_action(self, action_type, params=None):
        """
        Generic helper function for publishing GenericAction msg
        :param publisher: publisher to use
        :param action_type: the type of the action msg
        :param params: optional parameter for the msg
        """
        if params is None:
            params = []

        action = GenericAction()
        action.action_type = action_type
        action.params = params
        self._pub_generic_action.publish(action)
        self.current_action_sent = True

    def action_go_to_location(self, lat, lon):
        self.send_action(action_type=Action.GO_TO, params=[
            KeyValue("latitude", str(lat)),
            KeyValue("longitude", str(lon))
        ])

    def action_go_to_position(self, pos):
        """

        :param pos:
        :type pos: Position
        :return:
        """
        self.send_action(action_type=Action.GO_TO, params=[
            KeyValue("latitude", str(pos.lat)),
            KeyValue("longitude", str(pos.long))
        ])
