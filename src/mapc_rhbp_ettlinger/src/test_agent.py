import time

import rospy
from mac_ros_bridge.msg import Item, Position

from common_utils.agent_utils import AgentUtils
from coordination.assemble_contractor import AssembleContractor
from coordination.assemble_manager import AssembleManager
from provider.product_provider import ProductProvider


class TestAgent(object):

    def __init__(self):

        rospy.init_node('planner_node', anonymous=True, log_level=rospy.ERROR)
        self.assemble_manager = AssembleManager(
            agent_name="agentA1",
            role="drone"
        )
        self.assemble_contractor_0 = AssembleContractor(
            agent_name="agentA1",
            role="drone",
            product_provider=FakeProductProvider(
                agent_name="agentA1",
                items=[
                    Item(
                        name="item1",
                        amount=3)
                ]
            )
        )
        self.assemble_contractor_1 = AssembleContractor(
            agent_name="agentA2",
            role="truck",
            product_provider=FakeProductProvider(
                agent_name="agentA2",
                items=[
                    Item(
                        name="item0",
                        amount=4)
                ]
            )
        )
        self.assemble_contractor_2 = AssembleContractor(
            agent_name="agentA3",
            role="motorcycle",
            product_provider=FakeProductProvider(
                agent_name="agentA3",
                items=[
                    Item(
                        name="item3",
                        amount=3)
                ]
            ))
        self.assemble_contractor_3 = AssembleContractor(
            agent_name="agentA4",
            role="car",
            product_provider=FakeProductProvider(
                agent_name="agentA4",
                items=[
                    Item(
                        name="item2",
                        amount=7)
                ]
            ))
        self.assemble_contractor_4 = AssembleContractor(
            agent_name="agentA5",
            role="motorcycle",
            product_provider=FakeProductProvider(
                agent_name="agentA5",
                items=[
                    Item(
                        name="item4",
                        amount=5)
                ]
            ))

        self._agent_topic_prefix = AgentUtils.get_bridge_topic_prefix(agent_name="agentA1")

        time.sleep(1)
        rospy.logerr("Request assist")
        self.assemble_manager.request_assist(Position(lat=48.82456, long=2.31017))

class FakeProductProvider(ProductProvider):

    def __init__(self, agent_name, items):
        self.items = items
        super(FakeProductProvider, self).__init__(agent_name=agent_name)

    def get_items(self):
        return self.items



if __name__ == '__main__':

    try:

        # Just take a random agent. doesn't really matter
        job_planner = TestAgent()

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
