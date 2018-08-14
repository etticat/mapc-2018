from common_utils.agent_utils import AgentUtils
from my_subscriber import MyPublisher
from so_data.sobroadcaster import SoBroadcaster


class MassimSoBroadcaster(SoBroadcaster):


    def init_publisher(self):
        topic = AgentUtils.get_coordination_topic()
        return MyPublisher(topic, message_type="so_data", task_type="so_data", queue_size=10)
