import rospy

from behaviour_components.activators import ThresholdActivator
from behaviour_components.behaviours import BehaviourBase
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition, Negation, Conjunction
from behaviour_components.goals import GoalBase
from behaviour_components.network_behavior import NetworkBehaviour
from behaviours.job import GoToResourceBehaviour, GatherBehaviour, AssembleProductBehaviour, GoToWorkshopBehaviour
from provider.product_provider import ProductProvider
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

        self.init_choose_finished_products_behaviour(agent, proximity)
        self.init_go_to_resource_behaviour(agent, proximity)
        self.init_assembly_behaviour(agent)

        # The goal is to have a full storage (so we can use it to assemble)
        # self.fill_stock_up_goal = GoalBase(
        #     name='fill_up_stock',
        #     permanent=True,
        #     plannerPrefix=self.get_manager_prefix(),
        #     conditions=[Negation(self.next_item_fits_in_storage_condition)])

    def init_assembly_behaviour(self, agent):
        ############### Assembling ##########################
        self.assemble_product_behaviour = AssembleProductBehaviour(
            name="assemble_product_behaviour",
            agent_name=agent._agent_name,
            plannerPrefix=self.get_manager_prefix(),
            behaviour_name=self.go_to_workshop_behaviour._name,
            product_provider_method=self._product_provider.get_required_finished_products_for_hoarding
        )
        # Only assemble if we are at the intended resource node
        self.assemble_product_behaviour.add_precondition(
            precondition=self.at_workshop_condition)

        # only assemble if we have chosen an item to assemble
        self.assemble_product_behaviour.add_precondition(
            precondition=Negation(self.has_all_finished_products_condition)
        )

        # TODO: Add a proper effect
        self.assemble_product_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.remaining_finished_products_sensor.name,
                indicator=-1.0,
                sensor_type=float

            )
        )

    def init_go_to_resource_behaviour(self, agent, proximity):
        ################ Going to Workshop #########################
        self.go_to_workshop_behaviour = GoToWorkshopBehaviour(
            agent=agent,
            agent_name=agent._agent_name,
            name="go_to_workshop",
            plannerPrefix=self.get_manager_prefix(),
            topic="/workshop",
            product_provider_method=self._product_provider.get_required_finished_products_for_hoarding

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
            precondition=Negation(self.has_all_finished_products_condition)
        )

        # Going to resource has the effect of decreasing the distance to go there
        self.go_to_workshop_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.workshop_destination_sensor.name,
                indicator=-1.0,
                sensor_type=float

            )
        )
    def init_product_sensor(self, agent):
        self.remaining_finished_products_sensor = ProductSensor(
            name="remaining_finished_products_sensor",
            agent_name=agent._agent_name,
            product_provider_method=self._product_provider.get_required_finished_products_for_hoarding)

        self.has_all_finished_products_condition = Condition(
            sensor=self.remaining_finished_products_sensor,
            activator=AmountInListActivator(
                amount=0
            ))

    def stop(self):
        """
        Stoping all gather goals when leaving network
        :return:
        """
        self._product_provider.stop_assembly()
        super(AssembleNetworkBehaviour, self).stop()

    def init_choose_finished_products_behaviour(self, agent, proximity):
        self.choose_finished_products_behaviour = ChooseFinishedProductsBehaviour(
            name="choose_finished_products_behaviour",
            agent_name=agent._agent_name,
            plannerPrefix=self.get_manager_prefix()
        )

        # only chose an item if we currently don't have a goal
        self.choose_finished_products_behaviour.add_precondition(
            precondition=self.has_all_finished_products_condition
        )

        # Chosing an ingredient has the effect, that we have more ingredients to gather
        self.choose_finished_products_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.remaining_finished_products_sensor.name,
                indicator=1.0,
                sensor_type=bool
            )
        )


class ChooseFinishedProductsBehaviour(BehaviourBase):

    def __init__(self, agent_name, **kwargs):
        super(ChooseFinishedProductsBehaviour, self).__init__(**kwargs)

        self._product_provider = ProductProvider(agent_name=agent_name)

    def do_step(self):
        # TODO: Make this super elaborate
        # - Check what products we currently have few of
        # - Check which products usually result in most money
        # - Check which ingredients other agents currently hold, how far they are way, ... (maybe negotiate with them) and invite those
        # - Decide who is the assembler (who gets the item)
        # - ...
        # - Look into protocols that help with this task
        # - put everything into a seperate file (or multiple)

        self._product_provider.start_assembly({
            "item5":0,
            "item6":0,
            "item7":0,
            "item8":0
        })
