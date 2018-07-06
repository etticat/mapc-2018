from mac_ros_bridge.msg import Agent

import rospy
from common_utils import etti_logging

from common_utils.agent_utils import AgentUtils
from decisions.gathering import ChooseIngredientToGather
from provider.action_provider import ActionProvider
from provider.distance_provider import DistanceProvider
from provider.facility_provider import FacilityProvider
from provider.product_provider import ProductProvider
from provider.simulation_provider import SimulationProvider
from so_data.patterns import DecisionPattern


ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.decisions.gathering')


class ChooseResourceMechanism(DecisionPattern):

    def __init__(self, agent_name):

        self.agent_name = agent_name
        self._last_agent_position = None
        self._last_calculated_agent_position = None

        self.facility_provider = FacilityProvider()
        self.distance_provider = DistanceProvider()

        self.action_provider = ActionProvider(agent_name=agent_name)
        self._product_provider = ProductProvider(agent_name=agent_name)
        self._choose_item = ChooseIngredientToGather(agent_name=agent_name)
        self.chosen_resource = None
        super(ChooseResourceMechanism, self).__init__(buffer=None, frame=None, requres_pos=False)

    def calc_value(self):
        if self.chosen_resource is None or not self._product_provider.fits_in_store(self.chosen_resource.item.name):
            resource = self._choose_item.choose_resource()
            # TODO: Pull in fuctionality

            if resource is not None:
                if resource.item is not None:
                    self.chosen_resource = resource
                    self._product_provider.start_gathering(resource.item.name)
                    ettilog.logerr("ChooseResourceMechanism(%s):: Choosing item %s", self.agent_name, resource.item)
                    return [resource, self.state]
                else:
                    ettilog.logerr("ChooseResourceMechanism(%s):: Trying to choose item, but none fit in stock",
                                   self.agent_name)

            else:
                ettilog.logerr(
                    "ChooseResourceMechanism(%s):: Trying to choose item, but no resource chosen",
                    self.agent_name)
        else:
            return [self.chosen_resource, self.state]

    def end_gathering(self):
        self._product_provider.stop_gathering()
        self.chosen_resource = None