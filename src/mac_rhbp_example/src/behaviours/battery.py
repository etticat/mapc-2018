import rospy

from agent_common.agent_utils import AgentUtils
from behaviours.generic_action import GenericActionBehaviour, Action
from behaviours.movement import GotoFacilityBehaviour
from utils.ros_helpers import get_topic_type


class GoToChargingstationBehaviour(GotoFacilityBehaviour):

    def __init__(self, agent,plannerPrefix,  **kwargs):
        topic = '/charging_station'
        self.agent = agent
        super(GoToChargingstationBehaviour,self).__init__(
            agent._agent_name,
            topic,
            plannerPrefix=plannerPrefix,
            name="go_to_charging_behaviour",
            graph_name="charging",
            **kwargs)

        facility_topic_type = get_topic_type(topic)

        self._facilities = {}
        rospy.Subscriber(topic , facility_topic_type, self._callback_facility)


    def _callback_facility(self, msg):
        # Store all available facilities in a dict
        for facility in msg.facilities:
            self._facilities[facility.name] = facility

    def _select_pos(self):
        """
        Determine the facility we want to go to, using euclidean distance
        TODO: Use graphhopper for non drone vehicles
        :return: facility
        """
        closest_facility = None
        min_distance = 9999

        for _, facility in self._facilities.items():
            distance = AgentUtils.euclidean_distance(self.agent.agent_info.pos, facility.pos)
            if distance < min_distance:
                min_distance = distance
                closest_facility = facility

        return closest_facility.pos

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
                      agent_name = agent_name,
                      action_type = Action.RECHARGE,
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
                      agent_name = agent_name,
                      action_type = Action.CHARGE,
                      **kwargs)
