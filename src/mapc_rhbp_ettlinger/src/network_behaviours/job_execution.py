from behaviours.job import DeliverJobBehaviour
from decisions.p_task_decision import CurrentTaskDecision
from network_behaviours.go_and_do import GoAndDoNetworkBehaviour
from sensor.sensor_map import SensorAndConditionMap


class DeliverJobNetworkBehaviour(GoAndDoNetworkBehaviour):

    def __init__(self, agent_name, name, sensor_map, **kwargs):
        """

        :param agent_name:
        :param name:
        :param sensor_map:
        :type sensor_map: SensorAndConditionMap
        :param kwargs:
        """

        super(DeliverJobNetworkBehaviour, self).__init__(
            agent_name=agent_name,
            sensor_map=sensor_map,
            name=name,
            mechanism=sensor_map.deliver_task_mechanism,
            **kwargs)

        deliver_job_behaviour = DeliverJobBehaviour(
            name="deliver_job_behaviour",
            agent_name=agent_name,
            mechanism=sensor_map.deliver_task_mechanism,
            plannerPrefix=self.get_manager_prefix()
        )
        self.init_do_behaviour(deliver_job_behaviour)