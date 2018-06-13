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
from coordination.assemble_manager import AssembleManager
from provider.product_provider import ProductProvider
from rhbp_utils.knowledge_sensors import KnowledgeSensor
from sensor.agent import StorageAvailableForItemSensor
from sensor.job import ProductSensor, AmountInListActivator
from sensor.movement import DestinationDistanceSensor


class AssembleNetworkBehaviour(NetworkBehaviour):

    def __init__(self, agent, name, msg, **kwargs):

        proximity = msg.proximity
        self._product_provider = ProductProvider(
            agent_name=agent._agent_name)

        super(AssembleNetworkBehaviour, self).__init__(name, **kwargs)

        self.init_product_sensor(agent)

        self.init_choose_finished_products_behaviour(agent, msg.role)
        self.init_go_to_workshop_behaviour(agent, proximity)
        self.init_assembly_behaviour(agent)

        # The goal is to have a full storage (so we can use it to assemble)
        # self.fill_stock_up_goal = GoalBase(
        #     name='fill_up_stock',
        #     permanent=True,
        #     plannerPrefix=self.get_manager_prefix(),
        #     conditions=[Negation(self.)])

    def init_assembly_behaviour(self, agent):
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
            precondition=self.has_assemble_task_assigned_cond
        )

        # TODO: Add a proper effect
        self.assemble_product_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.assemble_organized_sensor.name,
                indicator=-1.0,
                sensor_type=float

            )
        )

    def init_go_to_workshop_behaviour(self, agent, proximity):
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
            precondition=self.has_assemble_task_assigned_cond)

        # Going to resource has the effect of decreasing the distance to go there
        self.go_to_workshop_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.workshop_destination_sensor.name,
                indicator=-1.0,
                sensor_type=float

            )
        )
    def init_product_sensor(self, agent):
        self.assemble_organized_sensor = KnowledgeSensor(
            name="assemble_organized_sensor",
            pattern=AssembleKnowledgebase.generate_tuple(
                agent_name=agent._agent_name,
                active=True))

        self.has_assemble_task_assigned_cond = Condition(
            sensor=self.assemble_organized_sensor,
            activator=BooleanActivator(
                desiredValue=True
            ))

    def stop(self):
        """
        Stoping all gather goals when leaving network
        :return:
        """
        self._product_provider.stop_assembly()
        super(AssembleNetworkBehaviour, self).stop()

    def init_choose_finished_products_behaviour(self, agent, role):
        self.choose_finished_products_behaviour = CoordinateAssemblyBehaviour(
            name="choose_finished_products_behaviour",
            agent_name=agent._agent_name,
            plannerPrefix=self.get_manager_prefix(),
            role=role.name
        )

        # only chose an item if we currently don't have a goal
        self.choose_finished_products_behaviour.add_precondition(
            precondition=Negation(self.has_assemble_task_assigned_cond)
        )

        # Chosing an ingredient has the effect, that we have more ingredients to gather
        self.choose_finished_products_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.assemble_organized_sensor.name,
                indicator=1.0,
                sensor_type=bool
            )
        )


class CoordinateAssemblyBehaviour(BehaviourBase):

    def __init__(self, agent_name, role, **kwargs):
        super(CoordinateAssemblyBehaviour, self).__init__(requires_execution_steps=True, **kwargs)

        self._agent_name = agent_name
        self._assemble_manager = AssembleManager(agent_name=agent_name, role=role)

    def do_step(self):
        # TODO: Make this super elaborate
        # - Check which products usually result in most money
        # - Decide who is the assembler (who gets the item)
        # - ...

        # if self._agent_name in ["agentA13", "agentA1"]: # Temporarily for testing. only 1 and 13 are managers
        self._assemble_manager.request_assist() # TODO: find closest one
        # else:
            # rospy.logerr("%s: manager busy", self._name)
        pass