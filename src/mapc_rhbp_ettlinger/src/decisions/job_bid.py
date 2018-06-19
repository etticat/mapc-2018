import random

from mapc_rhbp_ettlinger.msg import JobBid

from common_utils.calc import CalcUtil
from provider.distance_provider import DistanceProvider
from provider.product_provider import ProductProvider


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
        own_items = CalcUtil.get_list_from_items(self._product_provider.get_items())
        item_intersect = CalcUtil.list_intersect(request.items, own_items)
        if len(item_intersect) > 0 or len(request.items) == 0:
            return JobBid(
                id=request.id,
                bid=random.randint(0, 7),  # TODO
                expected_steps=random.randint(3, 10),  # TODO
                agent_name=self._agent_name,
                items=item_intersect,
            )
        else:
            return None
