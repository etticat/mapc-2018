from __future__ import division

from behaviour_components.behaviours import BehaviourBase
from common_utils import etti_logging
from provider.action_provider import ActionProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.behaviours.generic')

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

        self.action_provider = ActionProvider(agent_name=agent_name)
        self._agent_name = agent_name

        self._action_type = action_type
        self._params = params

    def do_step(self):
        ettilog.logdebug(self._agent_name + "::" + self._name + " executing: " + self._action_type)
        self.action_provider.send_action(action_type=self._action_type, params=self.generate_params())


    def generate_params(self):
        return self._params
