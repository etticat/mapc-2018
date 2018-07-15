from behaviours.job import DeliverJobBehaviour
from network_behaviours.go_and_do import GoAndDoNetworkBehaviour


class DeliverJobNetworkBehaviour(GoAndDoNetworkBehaviour):
    """
    Network behaviour responsible for delivering items for job completion
    """

    def __init__(self, agent_name, name, global_rhbp_components, **kwargs):

        super(DeliverJobNetworkBehaviour, self).__init__(
            agent_name=agent_name,
            global_rhbp_components=global_rhbp_components,
            name=name,
            mechanism=global_rhbp_components.deliver_task_mechanism,
            **kwargs)

        self.init_do_behaviour(DeliverJobBehaviour(
            name="deliver_job_behaviour",
            agent_name=agent_name,
            mechanism=global_rhbp_components.deliver_task_mechanism,
            plannerPrefix=self.get_manager_prefix()
        ))