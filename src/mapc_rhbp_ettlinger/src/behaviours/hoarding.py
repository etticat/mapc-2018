from diagnostic_msgs.msg import KeyValue
from mapc_rhbp_ettlinger.msg import Task

from behaviour_components.behaviours import BehaviourBase
from behaviours.generic_action import GenericActionBehaviour
from common_utils import etti_logging
from common_utils.calc import CalcUtil
from provider.action_provider import Action
from provider.distance_provider import DistanceProvider
from provider.facility_provider import FacilityProvider
from provider.product_provider import ProductProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.behaviours.hoarding')

class StoreBehaviour(GenericActionBehaviour):
    """
    A behaviour for triggering recharge actions which can be executed everywhere
    """

    def __init__(self, name, agent_name, mechanism,  **kwargs):
        """
        :param name: name of the behaviour
        :param agent_name: name of the agent for determining the correct topic prefix
        :param mechanism
        :param kwargs: more optional parameter that are passed to the bas class
        """
        super(StoreBehaviour, self) \
            .__init__(name=name,
                      agent_name=agent_name,
                      action_type=Action.STORE,
                      **kwargs)
        self.mechanism = mechanism
        self._task = None
        self._product_provider = ProductProvider(agent_name=agent_name)

    def generate_params(self):
        finished_products_to_store = self.mechanism.calc_value()[0]
        self._product_provider.update_hoarding_goal(finished_products_to_store, destination=self.mechanism.value.name)
        for item, count in finished_products_to_store.iteritems():
            if count > 0:
                ettilog.logerr("StoreBehaviour:: Storing %s(%d)", item, count)
                return [KeyValue(key="Item", value=item),KeyValue(key="Value", value=str(int(count)))]

        ettilog.logerr("StoreBehaviour:: ERROR: Trying to store, but no finished products available")