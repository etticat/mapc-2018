from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Negation
from behaviour_components.goals import GoalBase
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

        self.deliver_job_behaviour = DeliverJobBehaviour(name="deliver_job_behaviour", agent_name=agent_name,
                                        mechanism=global_rhbp_components.deliver_task_mechanism,
                                        plannerPrefix=self.get_manager_prefix())
        self.init_do_behaviour(self.deliver_job_behaviour)
        self.deliver_job_behaviour.add_effect(Effect(
            sensor_type=bool,
            sensor_name=global_rhbp_components.has_deliver_task_sensor.name,
            indicator=-1.0
        ))
        self.delivery_goal = GoalBase(
            name='delivery_goal',
            permanent=True,
            priority=50,
            planner_prefix=self._agent_name,
            conditions=[Negation(self._global_rhbp_components.has_deliver_job_task_assigned_cond)])