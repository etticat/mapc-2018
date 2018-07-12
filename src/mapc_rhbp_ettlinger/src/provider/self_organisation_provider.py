import numpy as np

import rospy

from common_utils.agent_utils import AgentUtils
from common_utils.singleton import Singleton
from provider.distance_provider import DistanceProvider
from self_organisation.massim_so_buffer import MassimSoBuffer
from self_organisation.posegradienttf import CallbackEntityTopicGradientTf
from so_data.msg import SoMessage
from so_data.sobuffer import SoBuffer


class SelfOrganisationProvider(object):
    __metaclass__ = Singleton

    def __init__(self, agent_name):
        self.distance_provider = DistanceProvider()
        self.value_key = 'value_' + agent_name
        self.frame_agent = "agent"
        self.agent_name = agent_name
        self.buffer = MassimSoBuffer(id=agent_name, pose_frame=self.frame_agent,
                               view_distance=np.inf, moving_storage_size=np.inf)

    def init_entity_listener(self):
        entity_topic = AgentUtils.get_bridge_topic(agent_name=self.agent_name, postfix="entity")
        # publish robot gradient data (transformation pose topic to soMessage)
        self.pose_tf = CallbackEntityTopicGradientTf(topic=entity_topic, frame=self.frame_agent,
                                                     id=self.agent_name)


    def send_msg(self, pos, frame, parent_frame, attraction=-1, diffusion=0.0, goal_radius=0.5, ev_factor=1.0,
                 ev_time=0, moving=False, payload=None, time=None):
        """
        creates soMessage with set parameters
        :return: gradient message / SoMessage
        """
        msg = SoMessage()

        # current time
        now = rospy.Time.now()

        msg.header.frame_id = frame
        msg.parent_frame = parent_frame
        msg.header.stamp = now
        if time is not None:
            msg.header.stamp.secs = time
        msg.p.x = self.distance_provider.lat_to_x(pos.lat)
        msg.p.y = self.distance_provider.lon_to_y(pos.long)
        msg.attraction = attraction
        msg.diffusion = diffusion
        msg.goal_radius = goal_radius
        msg.ev_factor = ev_factor
        msg.ev_time = ev_time
        msg.ev_stamp = now
        msg.moving = moving
        msg.payload = payload

        return msg
