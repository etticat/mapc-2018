import rospy
from mac_ros_bridge.msg import Position

from agent_knowledge.assemble_task import AssembleKnowledgebase
from behaviour_components.activators import ThresholdActivator, BooleanActivator
from behaviour_components.behaviours import BehaviourBase
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition, Negation, Conjunction
from behaviour_components.goals import GoalBase
from behaviour_components.network_behavior import NetworkBehaviour
from behaviours.job import GoToResourceBehaviour, GatherBehaviour, AssembleProductBehaviour, GoToWorkshopBehaviour
from network_coordination.assemble_manager import AssembleManager
from provider.product_provider import ProductProvider
from rhbp_utils.knowledge_sensors import KnowledgeSensor
from sensor.agent import StorageAvailableForItemSensor
from sensor.job import ProductSensor, AmountInListActivator
from sensor.movement import DestinationDistanceSensor


class AssembleNetworkBehaviour(NetworkBehaviour):

    def __init__(self, agent, name, msg, coordination_network_behaviour, **kwargs):

        proximity = msg.proximity
        self._product_provider = ProductProvider(
            agent_name=agent._agent_name)

        super(AssembleNetworkBehaviour, self).__init__(name, **kwargs)

        self.init_go_to_workshop_behaviour(agent, proximity, coordination_network_behaviour)
        self.init_assembly_behaviour(agent, coordination_network_behaviour)

        # The goal is to have a full storage (so we can use it to assemble)
        # self.fill_stock_up_goal = GoalBase(
        #     name='fill_up_stock',
        #     permanent=True,
        #     plannerPrefix=self.get_manager_prefix(),
        #     conditions=[Negation(self.)])

    def init_assembly_behaviour(self, agent, coordination_network_behaviour):
        ############### Assembling ##########################
        self.assemble_product_behaviour = AssembleProductBehaviour(
            name="assemble_product_behaviour",
            agent_name=agent._agent_name,
            plannerPrefix=self.get_manager_prefix(),
            behaviour_name=self.go_to_workshop_behaviour._name
        )
        # Only assemble if we are at the intended resource node
        self.assemble_product_behaviour.add_precondition(
            precondition=self.at_workshop_condition)

        # only assemble if we have chosen an item to assemble
        self.assemble_product_behaviour.add_precondition(
            precondition=coordination_network_behaviour.has_assemble_task_assigned_cond
        )

        # TODO: Add a proper effect
        self.assemble_product_behaviour.add_effect(
            effect=Effect(
                sensor_name=coordination_network_behaviour.assemble_organized_sensor.name,
                indicator=-1.0,
                sensor_type=float

            )
        )

    def init_go_to_workshop_behaviour(self, agent, proximity, coordination_network_behaviour):
        ################ Going to Workshop #########################
        self.go_to_workshop_behaviour = GoToWorkshopBehaviour(
            agent=agent,
            agent_name=agent._agent_name,
            name="go_to_workshop",
            plannerPrefix=self.get_manager_prefix(),
            # TODO: Remove this method and use the data from the contract net

        )
        self.workshop_destination_sensor = DestinationDistanceSensor(
            name='workshop_destination_sensor',
            agent_name=agent._agent_name,
            behaviour_name=self.go_to_workshop_behaviour._name)
        self.at_workshop_condition = Condition(
            sensor=self.workshop_destination_sensor,
            activator=ThresholdActivator(
                thresholdValue=proximity,
                isMinimum=False))  # highest activation if the value is below threshold
        # Only go to resource node if we aren't already there
        self.go_to_workshop_behaviour.add_precondition(
            precondition=Negation(self.at_workshop_condition))
        # only go to resource node if we have chosen an item to gather
        self.go_to_workshop_behaviour.add_precondition(
            precondition=coordination_network_behaviour.has_assemble_task_assigned_cond)

        # Going to resource has the effect of decreasing the distance to go there
        self.go_to_workshop_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.workshop_destination_sensor.name,
                indicator=-1.0,
                sensor_type=float

            )
        )

    def stop(self):
        """
        Stoping all gather goals when leaving network
        :return:
        """
        self._product_provider.stop_assembly()
        super(AssembleNetworkBehaviour, self).stop()
