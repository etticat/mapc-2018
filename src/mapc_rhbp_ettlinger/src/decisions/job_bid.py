import random

from mapc_rhbp_ettlinger.msg import TaskBid

from common_utils import etti_logging
from common_utils.calc import CalcUtil
from provider.distance_provider import DistanceProvider
from provider.product_provider import ProductProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.decisions.job_bid')

class JobBidDecider(object):

    def __init__(self, agent_name):
        self._product_provider = ProductProvider(agent_name=agent_name)
        self._agent_name = agent_name
        self._step_provider = DistanceProvider()

    def generate_bid(self, request):
        """

        :param request:
        :type request:
        :return: JobRequest
        """
        own_items = self._product_provider.get_item_list()
        item_intersect = CalcUtil.list_intersect(request.items, own_items)
        if len(item_intersect) > 0 or len(request.items) == 0:
            return TaskBid(
                id=request.id,
                agent_name=self._agent_name,
                expected_steps=random.randint(3, 10),  # TODO
                items=item_intersect,
                request=request
            )
        else:
            return None
