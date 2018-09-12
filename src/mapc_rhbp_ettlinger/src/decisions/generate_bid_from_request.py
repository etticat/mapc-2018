from mapc_rhbp_ettlinger.msg import TaskBid, TaskRequest

from common_utils import etti_logging
from common_utils.calc import CalcUtil
from provider.distance_provider import DistanceProvider
from provider.facility_provider import FacilityProvider
from provider.product_provider import ProductProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.decisions.job_bid')


class GenerateBidFromRequestDecision(object):
    """
    Creates a bid for a given job request
    """

    def __init__(self, agent_name):
        self._facility_provider = FacilityProvider(agent_name=agent_name)
        self._product_provider = ProductProvider(agent_name=agent_name)
        self._agent_name = agent_name
        self._step_provider = DistanceProvider(agent_name=agent_name)

    def generate_bid(self, request, has_assembly_task):
        """
        Creates the bid from the request
        :param has_assembly_task:
        :param request:
        :type request: TaskRequest
        :return: JobRequest
        """

        if has_assembly_task:
            bid = 0
        else:
            bid = 1

        own_items = self._product_provider.get_item_list()
        useful_items = CalcUtil.list_intersect(request.items, own_items)
        # Depends on how intensive I use hoarding in last version.
        if len(useful_items) > 0 or len(request.items) == 0:
            return TaskBid(
                id=request.id,
                bid=bid,
                agent_name=self._agent_name,
                expected_steps=self._step_provider.calculate_steps(
                    self._facility_provider.get_storage_by_name(request.destination_name).pos),
                items=useful_items,
                request=request
            )
        else:
            return None
