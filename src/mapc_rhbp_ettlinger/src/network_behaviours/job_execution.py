from agent_knowledge.task import TaskBaseKnowledge
from behaviours.job import DeliverJobBehaviour
from network_behaviours.go_and_do import GoAndDoNetworkBehaviour


class DeliverJobNetworkBehaviour(GoAndDoNetworkBehaviour):

    def __init__(self, agent_name, name, sensor_map, **kwargs):
        super(DeliverJobNetworkBehaviour, self).__init__(
            agent_name=agent_name,
            sensor_map=sensor_map,
            name=name,
            task_type=TaskBaseKnowledge.TYPE_DELIVER,
            **kwargs)

        deliver_job_behaviour = DeliverJobBehaviour(
            name="deliver_job_behaviour",
            agent_name=agent_name,
            plannerPrefix=self.get_manager_prefix()
        )
        self.init_do_behaviour(deliver_job_behaviour)