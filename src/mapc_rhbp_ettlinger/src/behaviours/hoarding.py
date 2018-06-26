from diagnostic_msgs.msg import KeyValue
from mapc_rhbp_ettlinger.msg import Task

from agent_knowledge.task import TaskKnowledgeBase
from behaviour_components.behaviours import BehaviourBase
from behaviours.generic_action import GenericActionBehaviour, Action
from common_utils import etti_logging
from provider.distance_provider import DistanceProvider
from provider.facility_provider import FacilityProvider
from provider.product_provider import ProductProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.behaviours.hoarding')


class ChooseStorageBehaviour(BehaviourBase):

    def __init__(self, agent_name, **kwargs):
        self.agent_name = agent_name
        super(ChooseStorageBehaviour, self).__init__(**kwargs)
        self._task_knowledge_base = TaskKnowledgeBase()
        self._facility_provider = FacilityProvider()
        self._distance_provider = DistanceProvider()

    def do_step(self):
        storages = self._facility_provider.get_storages().values()
        # TODO: Look at where is a good place to store
        facility = self._distance_provider.get_closest_facility(storages)

        # self._product_provider.start_gathering(resource.item.name)
        self._task_knowledge_base.create_task(Task(
            type=TaskKnowledgeBase.TYPE_HOARDING,
            agent_name=self.agent_name,
            pos=facility.pos
        ))
        ettilog.loginfo("ChooseStorageBehaviour:: Choosing storage %s", facility.name)


class StoreBehaviour(GenericActionBehaviour):
    """
    A behaviour for triggering recharge actions which can be executed everywhere
    """

    def __init__(self, name, agent_name, **kwargs):
        """
        :param name: name of the behaviour
        :param agent_name: name of the agent for determining the correct topic prefix
        :param kwargs: more optional parameter that are passed to the bas class
        """
        super(StoreBehaviour, self) \
            .__init__(name=name,
                      agent_name=agent_name,
                      action_type=Action.STORE,
                      **kwargs)
        self._task = None
        self._product_provider = ProductProvider(agent_name=agent_name)

    def generate_params(self):
        finished_product_dict = self._product_provider.get_finished_products_in_stock()

        for item, count in finished_product_dict.iteritems():
            if count > 0:
                ettilog.logerr("StoreBehaviour:: Storing %s(%d)", item, count)
                return [KeyValue(key="Item", value=item),KeyValue(key="Value", value=str(count))]

        ettilog.logerr("StoreBehaviour:: ERROR: Trying to store, but no finished products available")