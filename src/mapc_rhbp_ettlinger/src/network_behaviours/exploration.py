from mapc_rhbp_ettlinger.msg import Movement

from agent_knowledge.movement import MovementKnowledgebase
from behaviour_components.activators import ThresholdActivator, GreedyActivator, LinearActivator, BooleanActivator
from behaviour_components.behaviours import BehaviourBase
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Negation, Condition
from behaviour_components.goals import GoalBase
from behaviours.movement import GotoLocationBehaviour2
from network_behaviours.battery import BatteryChargingNetworkBehaviour
from provider.simulation_provider import SimulationProvider
from rhbp_utils.knowledge_sensors import KnowledgeSensor
from sensor.exploration import ResourceDiscoveryProgressSensor
from sensor.movement import StepDistanceSensor, SelectedTargetPositionSensor


class ExplorationNetworkBehaviour(BatteryChargingNetworkBehaviour):
    def __init__(self, agent, msg, name, sensor_map, **kwargs):
        super(ExplorationNetworkBehaviour, self).__init__(name=name, agent=agent, msg=msg, sensor_map=sensor_map, **kwargs)


        self.init_resource_sensor(agent)
        self.init_destination_step_sensor(agent_name=agent._agent_name, sensor_map=sensor_map)

        self.init_choose_destination_behaviour(agent)
        self.init_go_to_destination_behaviour(agent, sensor_map)

        self._last_agent_pos = None

        self.exploration_goal = GoalBase(
            name='exploration_goal',
            permanent=True,
            plannerPrefix=self.get_manager_prefix(),
            conditions=[Condition(
                sensor=self.resource_discovery_progress_sensor,
                activator=GreedyActivator()
            )])

    def init_choose_destination_behaviour(self, agent):
        ####################### CHOOSE DESTINATION BEHAVIOUR ###################
        self.choose_destination_behaviour = ChooseDestinationBehaviour(
            plannerPrefix=self.get_manager_prefix(),
            name='choose_destination_behaviour',
            identifier=MovementKnowledgebase.IDENTIFIER_EXPLORATION,
            agent_name=agent._agent_name
        )
        self.choose_destination_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.target_step_sensor.name,
                indicator=1.0,  # Choosing a new destination increases the distance to the target
                sensor_type=float))
        # self.choose_destination_behaviour.add_effect(
        #     effect=Effect(
        #         sensor_name=self.exploration_target_exists_sensor.name,
        #         indicator=1.0,  # Choosing a new destination increases the distance to the target
        #         sensor_type=bool)) # TODO #86 How do I use this for a sensor with return value pos

        # Maybe use this: only allow picking a new one when the target is reached
        # self.choose_destination_behaviour.add_precondition(
        #     precondition=self.at_shop_cond)  # only finish if we are at a charging station


        self.has_gathering_task_sensor = KnowledgeSensor(
            name="has_exploration_task_sensor",
            pattern=MovementKnowledgebase.generate_tuple(
                agent_name=agent._agent_name,
                identifier=MovementKnowledgebase.IDENTIFIER_EXPLORATION
            )
        )
        self.has_gathering_task_cond = Condition(
            sensor=self.has_gathering_task_sensor,
            activator=BooleanActivator(desiredValue=True))

        self.choose_destination_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.has_gathering_task_sensor.name,
                indicator=1.0,
                sensor_type=bool))


    def init_go_to_destination_behaviour(self, agent, sensor_map):
        ####################### GO TO DESTINATION BEHAVIOUR ###################
        self._go_to_exploration_target_behaviour = GotoLocationBehaviour2(
            agent_name=agent._agent_name,
            identifier=MovementKnowledgebase.IDENTIFIER_EXPLORATION,
            plannerPrefix=self.get_manager_prefix(),
            name='go_to_exploration_target',
        )
        # exploration increases the nr of resources we know
        self._go_to_exploration_target_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.resource_discovery_progress_sensor.name,
                indicator=0.01,  # This should be less right? TODO #86
                sensor_type=float))
        # exploration has the effect that we well be at the place at some point
        self._go_to_exploration_target_behaviour.add_effect(
            effect=Effect(
                sensor_name=sensor_map.charge_sensor.name,
                indicator=-1.0,
                sensor_type=float))
        # Going to Shop gets us 1 step closer to the shop
        self._go_to_exploration_target_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.target_step_sensor.name,
                indicator=-1.0,
                sensor_type=float))

        # We can only walk to shop until we are there
        self._go_to_exploration_target_behaviour.add_precondition(
            precondition=Negation(self.at_shop_cond)
        )      # We can only walk to shop until we are there
        self._go_to_exploration_target_behaviour.add_precondition(
            precondition=Negation(self.has_gathering_task_cond)
        )

        self._go_to_exploration_target_behaviour.add_precondition(
            precondition=sensor_map.enough_battery_to_move_cond
        )

        # We can only walk to destination, if there is one already chosen
        # self._go_to_exploration_target_behaviour.add_precondition(
        #     precondition=self.has_exploration_target_cond)

    def init_destination_step_sensor(self, agent_name, sensor_map):
        self.exploration_target_sensor = SelectedTargetPositionSensor(
            identifier=MovementKnowledgebase.IDENTIFIER_EXPLORATION,
            name="exploration_target_sensor",
            agent_name=agent_name
        )

        # Sensor to check distance to charging station
        self.target_step_sensor = StepDistanceSensor(
            name='target_step_sensor',
            position_sensor_1=sensor_map.agent_position_sensor,
            position_sensor_2=self.exploration_target_sensor,
            initial_value=0
        )

        self.at_shop_cond = Condition(
            sensor=self.target_step_sensor,
            activator=ThresholdActivator(
                thresholdValue=0,
                isMinimum=False))

        # self.has_exploration_target_cond = Negation(Condition(
        #     sensor=self.exploration_target_sensor,
        #     activator=EqualActivator(
        #         desiredValue=None
        #     )
        # ))

    def init_resource_sensor(self, agent):
        self.resource_discovery_progress_sensor = ResourceDiscoveryProgressSensor(
            name="resource_discovery_progress_sensor",
            agent_name=agent._agent_name

        )
        self.discovery_completeness_condition = Condition(
            sensor=self.resource_discovery_progress_sensor,
            activator=LinearActivator(
                zeroActivationValue=0.0,
                fullActivationValue=1.0
            ))
        self.resources_of_all_items_discovered_condition = Condition(
            sensor=self.resource_discovery_progress_sensor,
            activator=ThresholdActivator(
                thresholdValue=1.0,
                isMinimum=True))


class ChooseDestinationBehaviour(BehaviourBase):
    def __init__(self, name, agent_name, identifier, **kwargs):
        super(ChooseDestinationBehaviour, self).__init__(name, requires_execution_steps=True, **kwargs)
        self.identifier = identifier
        self._simulation_provider = SimulationProvider()
        self._movement_knowledgebase = MovementKnowledgebase()

        self.agent_name = agent_name

    def do_step(self):
        destination = self._simulation_provider.get_random_position()
        self._movement_knowledgebase.start_movement(Movement(
            identifier = self.identifier,
            agent_name = self.agent_name,
            pos = destination
        ))