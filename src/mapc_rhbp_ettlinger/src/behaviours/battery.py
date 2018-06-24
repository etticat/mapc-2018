from behaviours.generic_action import GenericActionBehaviour, Action
from common_utils import etti_logging

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.behaviours.battery')


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
                      agent_name=agent_name,
                      action_type=Action.RECHARGE,
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
                      agent_name=agent_name,
                      action_type=Action.CHARGE,
                      **kwargs)
