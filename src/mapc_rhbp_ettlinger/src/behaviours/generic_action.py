from __future__ import division

import rospy
from mac_ros_bridge.msg import GenericAction

from behaviour_components.behaviours import BehaviourBase
from common_utils import rhbp_logging
from common_utils.agent_utils import AgentUtils

ettilog = rhbp_logging.LogManager(logger_name=rhbp_logging.LOGGER_DEFAULT_NAME + '.behaviours.generic')


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


class GenericActionBehaviour(BehaviourBase):
    """
    A simple behaviour for triggering generic MAC actions that just need a type and static parameters
    """

    def __init__(self, name, agent_name, action_type, params=[], **kwargs):
        """
        :param name: name of the behaviour
        :param agent_name: name of the agent for determining the correct topic prefix
        :param action_type: type of the MAC action
        :param params: optional parameters for the MAC action
        :param kwargs: more optional parameter that are passed to the bas class
        """
        super(GenericActionBehaviour, self) \
            .__init__(name=name,
                      requires_execution_steps=True, **kwargs)

        self._agent_name = agent_name

        self._action_type = action_type
        self._params = params
        self._pub_generic_action = rospy.Publisher(
            name=AgentUtils.get_bridge_topic_prefix(agent_name) + 'generic_action',
            data_class=GenericAction,
            queue_size=10)


    def do_step(self):
        ettilog.logdebug(self._agent_name + "::" + self._name + " executing: " + self._action_type)
        GenericActionBehaviour.action_generic_simple(publisher=self._pub_generic_action, action_type=self._action_type, params=self.generate_params())


    @staticmethod
    def action_generic(publisher, action_type, params=[]):
        """
        Generic helper function for publishing GenericAction msg
        :param publisher: publisher to use
        :param action_type: the type of the action msg
        :param params: optional parameter for the msg
        """
        action = GenericAction()
        action.action_type = action_type
        action.params = params

        ettilog.logdebug("Published action %s", action)

        publisher.publish(action)


    @staticmethod
    def action_generic_simple(publisher, action_type, params=[]):
        """
        Generic helper function for publishing GenericAction msg
        :param publisher: publisher to use
        :param action_type: the type of the action msg
        :param params: optional parameter for the msg
        """
        action = GenericAction()
        action.action_type = action_type
        action.params = params
        publisher.publish(action)

    def generate_params(self):
        return self._params