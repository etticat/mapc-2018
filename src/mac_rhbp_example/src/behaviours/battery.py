import rospy

from agent_common.agent_utils import AgentUtils
from behaviours.generic_action import GenericActionBehaviour, Action
from behaviours.movement import GotoLocationBehaviour

class RechargeBehaviour(GenericActionBehaviour):
    """
    A behaviour for triggering recharge actions which can be executed everywhere
    """

    def __init__(self, name, agent_name, **kwargs):
        """
        :param name: name of the behaviour
        :param agent_name: name of the agent for determining the correct topic prefix
        :param kwargs: more optional parameter that are passed to the bas class
        """
        super(RechargeBehaviour, self) \
            .__init__(name=name,
                      agent_name = agent_name,
                      action_type = Action.RECHARGE,
                      **kwargs)

class ChargeBehaviour(GenericActionBehaviour):
    """
    A behaviour for triggering charge actions which can be executed everywhere
    """

    def __init__(self, name, agent_name, **kwargs):
        """
        :param name: name of the behaviour
        :param agent_name: name of the agent for determining the correct topic prefix
        :param kwargs: more optional parameter that are passed to the bas class
        """
        super(ChargeBehaviour, self) \
            .__init__(name=name,
                      agent_name = agent_name,
                      action_type = Action.CHARGE,
                      **kwargs)
