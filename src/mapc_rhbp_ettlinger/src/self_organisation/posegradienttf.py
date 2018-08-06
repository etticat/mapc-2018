#! /usr/bin/env python2
"""

based on PoseTopicGradientTf by kaiser

Module to transform entity topic messages to SoMessages
"""

import copy

from mac_ros_bridge.msg import EntityMsg, Entity

from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from provider.distance_provider import DistanceProvider
from provider.simulation_provider import SimulationProvider
from so_data.topicgradienttf import TopicGradientTf

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.so.topic_gradient_tf')


class CallbackEntityTopicGradientTf(TopicGradientTf):
    """
    class to transform Topic messages mac_ros_bridge.msg.EntityMsg to soMessages
    """

    def __init__(self, agent_name, frame, id, message_type=EntityMsg,
                 **kwargs):
        """
        initialization
        :param topic: topic to subscribe to
        :param frame: header frame id
        :param id: soMessage id used for parent frame
        :param message_type: message type of topic
        :param kwargs: keyword arguments to specify other params of SoMessage
        """

        entity_topic = AgentUtils.get_bridge_topic(agent_name=agent_name, postfix="entity")

        self.distance_provider = DistanceProvider(agent_name=agent_name)
        self.simulation_provider = SimulationProvider(agent_name=agent_name)

        super(CallbackEntityTopicGradientTf, self).__init__(entity_topic, frame, id, message_type,
                                                            attraction=-1,
                                                            ev_factor=1.0,
                                                            moving=True, **kwargs)


    def callback(self, entity_msg):
        """
        extract inforamtion from entity messages
        :param entity_msg:
        :type entity_msg: EntityMsg
        :return:
        """

        # Track each entity we find
        for entity in entity_msg.entities:
            assert isinstance(entity, Entity)
            # deepcopy needed, otherwise leading to errors
            msg = copy.deepcopy(self.create_msg())

            # update data based on received information
            msg.p.x = self.distance_provider.lat_to_x(entity.pos.lat)
            msg.p.y = self.distance_provider.lon_to_y(entity.pos.long)
            msg.p.z = 0

            # Use the simulation step as time
            msg.ev_stamp.secs = self.simulation_provider.step

            # update self._current_gradient
            self._current_msg = msg

            if entity.name == self._id:
                # track ourselves
                msg.header.frame_id = "agent"
                ettilog.logdebug("EntityTopicGradientTf:: Found Agent: %s at %f, %f, %f", entity.name, msg.p.x, msg.p.y,
                                 msg.p.z)
                msg.diffusion = self.distance_provider._agent_vision

            elif entity.team is not self.simulation_provider.team:
                # track the other team
                msg.header.frame_id = "opponent"
                ettilog.logdebug("EntityTopicGradientTf:: Found Opponent: %s at %f, %f, %f", entity.name, msg.p.x,
                                 msg.p.y,
                                 msg.p.z)

                # We do not know the vision of the opponent. We estimate it and assume they do not upgrade.
                msg.diffusion = 600

            else:
                # In the case we se another of our team members we ignore them. they track themselves.
                ettilog.logdebug("EntityTopicGradientTf:: Ignoring team member: %s at %f, %f, %f", entity.name, msg.p.x,
                                 msg.p.y,
                                 msg.p.z)
                continue

            msg.parent_frame = entity.name

            # send gradient
            self.send()
