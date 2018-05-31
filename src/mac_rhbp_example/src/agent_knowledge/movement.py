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
    def get_movement_tuple(agent_name, behaviour="*", active="*", lat="*", long="*", destination="*"):
        return ('moving', behaviour, agent_name, str(active), str(lat), str(long), str(destination))

    INDEX_MOVEMENT_BEHAVIOUR = 1
    INDEX_MOVEMENT_AGENT_NAME = 2
    INDEX_MOVEMENT_ACTIVE = 3
    INDEX_MOVEMENT_LAT = 4
    INDEX_MOVEMENT_LONG = 5
    INDEX_MOVEMENT_DESTINATION = 6


    def start_movement(self, destinationPos, destination):
        search = MovementKnowledge.get_movement_tuple(self.agent_name, self.behaviour_name)
        new = MovementKnowledge.get_movement_tuple(self.agent_name, self.behaviour_name, active=True, lat=destinationPos.lat, long=destinationPos.long, destination=destination)

        rospy.logerr("MovementKnowledge:: Moving to %s ", destination)
        ret_value = self.__kb_client.update(search, new, push_without_existing = True)

    def stop_movement(self):
        search = MovementKnowledge.get_movement_tuple(self.agent_name, self.behaviour_name)
        new = MovementKnowledge.get_movement_tuple(self.agent_name, self.behaviour_name, active=False, lat="none", long="none", destination="none")


        rospy.logerr("MovementKnowledge:: Stopping movement")

        ret_value = self.__kb_client.update(search, new, push_without_existing = False)
        return ret_value

    def get_current_fact(self):

        search = MovementKnowledge.get_movement_tuple(self.agent_name, self.behaviour_name)
        facts = self.__kb_client.all(search)

        if len(facts) > 0:
            return facts[0]
        else:
            return None