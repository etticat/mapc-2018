import time

import rospy
from mac_ros_bridge.msg import Item, Position, Job

from common_utils.agent_utils import AgentUtils
from network_coordination.assemble_contractor import AssembleContractor
from network_coordination.assemble_manager import AssembleManager
from network_coordination.job_contractor import JobContractor
from network_coordination.job_manager import JobManager
from provider.product_provider import ProductProvider


class TestAgent(object):

    def __init__(self):

        rospy.init_node('planner_node', anonymous=True, log_level=rospy.ERROR)

        # self.test_assembly_contract_net()
        self.test_job_distribution_contract_net()

    def test_job_distribution_contract_net(self):
        self.job_manager = JobManager()

        self.job_contractor_0 = JobContractor(
            agent_name="agentA1",
            role="drone",
            product_provider=FakeProductProvider(
                agent_name="agentA1",
                items=[
                    Item(
                        name="item5",
                        amount=3)
                ]
            )
        )
        self.job_contractor_1 = JobContractor(
            agent_name="agentA2",
            role="truck",
            product_provider=FakeProductProvider(
                agent_name="agentA2",
                items=[
                    Item(
                        name="item5",
                        amount=6)
                ]
            )
        )
        self.job_contractor_2 = JobContractor(
            agent_name="agentA3",
            role="motorcycle",
            product_provider=FakeProductProvider(
                agent_name="agentA3",
                items=[
                    Item(
                        name="item8",
                        amount=4)
                ]
            ))
        self.job_contractor_3 = JobContractor(
            agent_name="agentA4",
            role="car",
            product_provider=FakeProductProvider(
                agent_name="agentA4",
                items=[
                    Item(
                        name="item4",
                        amount=3)
                ]
            ))
        self.job_contractor_4 = JobContractor(
            agent_name="agentA5",
            role="motorcycle",
            product_provider=FakeProductProvider(
                agent_name="agentA5",
                items=[
                    Item(
                        name="item4",
                        amount=4)
                ]
            ))
        self._agent_topic_prefix = AgentUtils.get_bridge_topic_prefix(agent_name="agentA1")
        time.sleep(1)
        rospy.logerr("Request assist")
        self.job_manager.job_request(Job(
            id = 'job12',
            storage_name = 'storage2',
            end = 0,
            items = [
                Item(
                    name="item5",
                    amount=6),
                Item(
                    name="item4",
                    amount=3)
            ]
        ))



    def test_assembly_contract_net(self):
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
        self.assemble_manager.request_assist()

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
