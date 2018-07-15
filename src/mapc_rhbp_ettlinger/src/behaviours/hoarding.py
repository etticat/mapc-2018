from diagnostic_msgs.msg import KeyValue

from behaviours.generic_action import GenericActionBehaviour
from common_utils import etti_logging
from provider.action_provider import Action
from provider.product_provider import ProductProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.behaviours.hoarding')


class StoreBehaviour(GenericActionBehaviour):
    """
    Behaviour to execute the store action to store items at a storage for later use in jobs
    """

    def __init__(self, name, agent_name, mechanism, **kwargs):
        """
        :param name: name of the behaviour
        :param agent_name: name of the agent for determining the correct topic prefix
        :param mechanism: The mechanism used to decide which items to store
        :type mechanism: ChooseFinishedProductsToStoreDecision
        :param kwargs: more optional parameter that are passed to the bas class
        """
        super(StoreBehaviour, self) \
            .__init__(name=name,
                      agent_name=agent_name,
                      action_type=Action.STORE,
                      **kwargs)

        self._mechanism = mechanism
        self._product_provider = ProductProvider(agent_name=agent_name)

    def generate_params(self):
        """
        Generates the params of the next items to store
        :return:
        """
        # Get all the items to store
        finished_products_to_store = self._mechanism.calc_value()[0]

        # Update the hoarding goal
        self._product_provider.update_hoarding_goal(finished_products_to_store, destination=self._mechanism.value.name)

        # If there is something to store, return the params for it
        for item, count in finished_products_to_store.iteritems():
            if count > 0:
                ettilog.loginfo("StoreBehaviour:: Storing %s(%d)", item, count)
                return [KeyValue(key="Item", value=item), KeyValue(key="Value", value=str(int(count)))]

        # If the hoarding was running while the we don't have anything to store, print an error
        ettilog.logerr("StoreBehaviour:: ERROR: Trying to store, but no finished products available")
