import time

import rospy
from mac_ros_bridge.msg import Item

from common_utils.agent_utils import AgentUtils
from coordination.assemble_contractor import AssembleContractor
from coordination.assemble_manager import AssembleManager


class TestAgent(object):

    def __init__(self):

        rospy.init_node('planner_node', anonymous=True, log_level=rospy.ERROR)
        self.assemble_manager = AssembleManager(agent_name="agentA1")
        self.assemble_manager.items = [
            Item(
                name="item1",
                amount=3)
        ]
        self.assemble_contractor_1 = AssembleContractor(agent_name="agentA2")
        self.assemble_contractor_1.items = [
            Item(
                name="item0",
                amount=4)
        ]
        self.assemble_contractor_2 = AssembleContractor(agent_name="agentA3")
        self.assemble_contractor_2.items = [
            Item(
                name="item3",
                amount=3)
        ]
        self.assemble_contractor_3 = AssembleContractor(agent_name="agentA4")
        self.assemble_contractor_3.items = [
            Item(
                name="item2",
                amount=7)
        ]
        self.assemble_contractor_4 = AssembleContractor(agent_name="agentA5")
        self.assemble_contractor_4.items = [
            Item(
                name="item4",
                amount=5)
        ]

        self._agent_topic_prefix = AgentUtils.get_bridge_topic_prefix(agent_name="agentA1")

        time.sleep(1)
        rospy.logerr("Request assist")
        self.assemble_manager.request_assist("workshop4")




if __name__ == '__main__':

    try:

        # Just take a random agent. doesn't really matter
        job_planner = TestAgent()

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
