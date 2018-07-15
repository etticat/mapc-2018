import numpy as np

import rospy

from common_utils.agent_utils import AgentUtils
from common_utils.singleton import Singleton
from provider.distance_provider import DistanceProvider
from self_organisation.massim_so_buffer import MassimSoBuffer
from self_organisation.posegradienttf import CallbackEntityTopicGradientTf
from so_data.msg import SoMessage

from so_data.sobroadcaster import SoBroadcaster
from so_data.sobuffer import SoBuffer


class SelfOrganisationProvider(object):
    """
    Self Organisation Provider holds the soBuffer, initialises the so listener and allows to send soMessages
    """
    __metaclass__ = Singleton

    def __init__(self, agent_name):
        self._agent_name = agent_name

        self._distance_provider = DistanceProvider(agent_name=agent_name)
        self._value_key = 'value_' + agent_name
        self._frame_agent = "agent"

        self._broadcast = SoBroadcaster()

        self._so_buffer = MassimSoBuffer(id=agent_name, pose_frame=self._frame_agent,
                                         view_distance=np.inf, moving_storage_size=np.inf, agent_name=agent_name)

    def init_entity_listener(self):
        """
        Initialises the EntityWatcher
        :return:
        """
        # publish robot gradient data (transformation pose topic to soMessage)
        self.pose_tf = CallbackEntityTopicGradientTf(agent_name=self._agent_name, frame=self._frame_agent,
                                                     id=self._agent_name)

    def send_msg(self, pos, frame, parent_frame, attraction=-1, diffusion=0.0, goal_radius=0.5, ev_factor=1.0,
                 ev_time=0, moving=False, payload=None, time=None):
        """
        creates soMessage with and publishes it
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
        msg.p.x = self._distance_provider.lat_to_x(pos.lat)
        msg.p.y = self._distance_provider.lon_to_y(pos.long)
        msg.attraction = attraction
        msg.diffusion = diffusion
        msg.goal_radius = goal_radius
        msg.ev_factor = ev_factor
        msg.ev_time = ev_time
        msg.ev_stamp = now
        msg.moving = moving
        msg.payload = payload

        self._broadcast.send_data(msg)

    @property
    def so_buffer(self):
        return self._so_buffer
