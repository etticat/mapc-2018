#!/usr/bin/env python2

import rospy
from knowledge_base.knowledge_base_client import KnowledgeBaseClient
from roswtf.roslaunchwtf import static_roslaunch_errors


class MovementKnowledge():

    def __init__(self, agent_name="*", behaviour_name="*"):
        self.__kb_client = KnowledgeBaseClient(
            knowledge_base_name = "knowledgeBaseNode")

        self.agent_name = agent_name
        self.behaviour_name = behaviour_name

    @staticmethod
    def get_knowledge_base_tuple_facility_movement(agent_name, goal,active="*"):
        return 'go_to_facility', agent_name, goal, str(active)

    @staticmethod
    def get_movement_tuple(agent_name, behaviour="*", active="*", lat="*", long="*"):
        return ('moving', agent_name, behaviour, str(active), str(lat), str(long))


    def start_movement(self, destination):
        search = MovementKnowledge.get_movement_tuple(self.agent_name, self.behaviour_name)
        new = MovementKnowledge.get_movement_tuple(self.agent_name, self.behaviour_name, active=True, lat=destination.lat, long=destination.long)

        ret_value = self.__kb_client.update(search, new, push_without_existing = True)

    def stop_movement(self):
        search = MovementKnowledge.get_movement_tuple(self.agent_name, self.behaviour_name)
        new = MovementKnowledge.get_movement_tuple(self.agent_name, self.behaviour_name, active=False, lat="none", long="none")

        rospy.logerr("stopping ..." )
        all_values = self.__kb_client.all(MovementKnowledge.get_movement_tuple(self.agent_name, self.behaviour_name ))
        for v in all_values:
            rospy.logerr(str(v))

        rospy.logerr(str(search))
        rospy.logerr(str(new))

        ret_value = self.__kb_client.update(search, new, push_without_existing = False)
        return ret_value

    def get_current_fact(self):

        search = MovementKnowledge.get_movement_tuple(self.agent_name, self.behaviour_name)
        facts = self.__kb_client.all(search)

        return facts