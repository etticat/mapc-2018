#!/usr/bin/env python2
from behaviour_components.behaviours import BehaviourBase
from behaviour_components.condition_elements import Effect
from behaviour_components.goals import GoalBase
from behaviour_components.managers import Manager
from behaviour_components.sensors import TopicSensor
from network_behaviours.assemble import AssembleNetworkBehaviour
from network_behaviours.build_well import BuildWellNetworkBehaviour
from network_behaviours.exploration import ExplorationNetworkBehaviour
from network_behaviours.gather import GatheringNetworkBehaviour
from network_behaviours.job_execution import DeliverJobNetworkBehaviour
from network_behaviours.sensor_map import SensorAndConditionMap


class ActionManager(Manager):
    def __init__(self, agent_name):
        self._agent_name = agent_name
        super(ActionManager, self).__init__(prefix=self._agent_name, max_parallel_behaviours=1)



    def init_behaviours(self, msg):
        self.massim_sensor = TopicSensor(
            topic="/team",
            name="massium_sensor",
            message_attr="massium")
        self.score_sensor = TopicSensor(
            topic="/team",
            name="score_sensor",
            message_attr="score")

        self.sensor_map = SensorAndConditionMap(agent_name=self._agent_name, msg=msg)

        self.init_behaviour_network()
        self.init_goals()
        self.init_coordination()
        # TODO: Add goals for assembly, massim, score, battery

    def init_behaviour_network(self):
        ####################### Exploration Network Behaviour ########################
        self.exploration_network = ExplorationNetworkBehaviour(
            name=self._agent_name + '/explore',
            plannerPrefix=self._agent_name,
            priority=1,
            agent_name=self._agent_name,
            sensor_map=self.sensor_map,
            max_parallel_behaviours=1)

        self.exploration_network.add_effect(
            Effect(
                sensor_name=self.sensor_map.resource_discovery_progress_sensor.name,
                indicator=1.0,
                sensor_type=float
            )
        )

        ######################## Gathering Network Behaviour ########################
        self._gathering_network = GatheringNetworkBehaviour(
            name=self._agent_name + '/gathering',
            plannerPrefix=self._agent_name,
            agent_name=self._agent_name,
            priority=2,
            sensor_map=self.sensor_map,
            max_parallel_behaviours=1)

        # Gather when there is space left in load
        # We want to gather till its full. so this doesn't really make sense
        # Maybe add some other condition to encourage it more when there is still a lot to gather
        # self._gathering_network.add_precondition(Negation(self.sensor_map.load_fullnes_condition))

        # Gather when we know some of the resource nodes already
        self._gathering_network.add_precondition(self.sensor_map.discovery_completeness_condition)

        # Gather only when next item fits in storage
        self._gathering_network.add_precondition(self.sensor_map.can_fit_more_ingredients_cond)

        self._gathering_network.add_effect(
            Effect(
                sensor_name=self.sensor_map.load_factor_sensor.name,
                indicator=1.0,
                sensor_type=float
            ))

        ####################### Assembly Network Behaviour ########################
        self._assembly_network = AssembleNetworkBehaviour(
            name=self._agent_name + '/assemble',
            plannerPrefix=self._agent_name,
            sensor_map=self.sensor_map,
            priority=4,
            agent_name=self._agent_name,
            max_parallel_behaviours=1)

        self._assembly_network.add_precondition(
            self.sensor_map.has_assemble_task_assigned_cond
        )

        self._assembly_network.add_effect(
            Effect(
                sensor_name=self.sensor_map.has_task_sensor.name,
                indicator=-1.0,
                sensor_type=bool
            )
        )
        ######################## Job Network Behaviour ########################
        self._job_execution_network = DeliverJobNetworkBehaviour(
            name=self._agent_name + '/job',
            plannerPrefix=self._agent_name,
            sensor_map=self.sensor_map,
            priority=5,
            agent_name=self._agent_name,
            max_parallel_behaviours=1)

        self._job_execution_network.add_precondition(
            self.sensor_map.has_deliver_job_task_assigned_cond
        )

        self._job_execution_network.add_effect(
            Effect(
                sensor_name=self.sensor_map.has_task_sensor.name,
                indicator=-1.0,
                sensor_type=bool
            )
        )

        ####################### Build Well Behaviour ########################
        self.build_well_network = BuildWellNetworkBehaviour(
            name=self._agent_name + '/well',
            plannerPrefix=self._agent_name,
            agent_name=self._agent_name,
            priority=3,
            sensor_map=self.sensor_map,
            max_parallel_behaviours=1)

        self.build_well_network.add_precondition(
            self.sensor_map.has_build_well_task_assigned_cond
        )

        self.build_well_network.add_effect(
            Effect(
                sensor_name=self.sensor_map.has_task_sensor.name,
                indicator=-1.0,
                sensor_type=bool
            )
        )


    def init_goals(self):

        self.task_fulfillment_goal = GoalBase(
            name='task_fulfillment_goal',
            permanent=True,
            priority=200,
            plannerPrefix=self._agent_name,
            conditions=[self.sensor_map.has_no_task_assigned_cond])

        self._gather_goal = GoalBase(
            name='fill_load_goal',
            permanent=True,
            priority=50,
            plannerPrefix=self._agent_name,
            conditions=[self.sensor_map.load_fullnes_condition])



    def init_coordination(self):
        # self.request_assembly = RequestAssemblyBehaviour(
        #     requires_execution_steps=False,
        #     name="request_assembly",
        #     plannerPrefix=self._agent_name)
        #
        # self.request_assembly.add_precondition(Negation(self._assembly_network.has_assemble_task_assigned_cond))
        #
        # self.request_assembly.add_effect(
        #     effect=Effect(
        #         sensor_name=self._assembly_network.assemble_organized_sensor.name,
        #         indicator=1.0,
        #         sensor_type=bool
        #     )
        # )
        pass


class RequestAssemblyBehaviour(BehaviourBase):
    pass