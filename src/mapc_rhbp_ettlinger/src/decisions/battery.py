from mac_ros_bridge.msg import Agent

import rospy

from common_utils.agent_utils import AgentUtils
from provider.distance_provider import DistanceProvider
from provider.facility_provider import FacilityProvider
from so_data.patterns import DecisionPattern


class ClosestChargingStationDecision(DecisionPattern):

    def __init__(self, agent_name):

        self.agent_name = agent_name
        self._last_agent_position = None
        self._last_calculated_agent_position = None

        self.facility_provider = FacilityProvider()
        self.distance_provider = DistanceProvider()

        super(ClosestChargingStationDecision, self).__init__(buffer=None, frame=None, requres_pos=False)

        self._sub_ref = rospy.Subscriber(AgentUtils.get_bridge_topic_agent(agent_name), Agent,
                                         self.subscription_callback_ref_topic)

    def subscription_callback_ref_topic(self, msg):
        if msg.pos != self._last_agent_position:
            self._last_agent_position = msg.pos

    def calc_value(self):
        if self._last_calculated_agent_position != self._last_agent_position:
            charging_stations = self.facility_provider.get_charging_stations()
            closest_facility = self.distance_provider.get_closest_facility(agent_position=self._last_agent_position,
                                                                           facilities=charging_stations)

            return [closest_facility, self.state]

        else:
            return [self.value, self.state]