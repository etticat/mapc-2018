import numpy as np

from common_utils.agent_utils import AgentUtils
from common_utils.singleton import Singleton
from provider.distance_provider import DistanceProvider
from self_organisation.massim_so_buffer import MassimSoBuffer
from self_organisation.posegradienttf import CallbackEntityTopicGradientTf
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
