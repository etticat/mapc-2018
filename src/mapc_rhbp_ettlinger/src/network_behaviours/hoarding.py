import rospy

from behaviour_components.activators import ThresholdActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition, Negation, Conjunction
from behaviour_components.goals import GoalBase
from behaviour_components.network_behavior import NetworkBehaviour
from behaviours.job import GoToResourceBehaviour, GatherBehaviour, AssembleProductBehaviour, GoToWorkshopBehaviour
from provider.product_provider import ProductProvider
from sensor.agent import StorageAvailableForItemSensor
from sensor.job import ProductSensor, AmountInListActivator
from sensor.movement import DestinationDistanceSensor


class HoardingNetworkBehaviour(NetworkBehaviour):

    def __init__(self, agent, name, msg, **kwargs):

        proximity = msg.proximity
        self._product_provider = ProductProvider(
            agent_name=agent._agent_name)

        super(HoardingNetworkBehaviour, self).__init__(name, **kwargs)

        self.init_product_sensors(agent)

        self.init_go_to_resource_behaviour(agent, proximity)
        self.init_gather_behaviour(agent)

        # TODO: this will be changed to int
        self.fill_stock_up_goal = GoalBase(
            name='fill_up_stock',
            permanent=True,
            plannerPrefix=self.get_manager_prefix(),
            conditions=[self.has_all_finished_products_condition])

    def init_gather_behaviour(self, agent):
        ############### Gathering ##########################
        self.gather_ingredients_behaviour = GatherBehaviour(
            name="gather_for_hoarding_behaviour",
            agent_name=agent._agent_name,
            plannerPrefix=self.get_manager_prefix(),
            behaviour_name=self.go_to_resource_node_behaviour._name,
            product_provider_method=self._product_provider.get_required_ingredients_for_hoarding
        )
        # Only gather if we are at the intended resource node
        self.gather_ingredients_behaviour.add_precondition(
            precondition=self.at_resource_node_condition)
        # Sensor to check how much space there is left after gaining the next intended item
        self.next_item_fits_in_storage_sensor = StorageAvailableForItemSensor(
            name="space_available_in_stock_sensor",
            agent_name=agent._agent_name,
            behaviour_name=self.go_to_resource_node_behaviour.name
        )
        # Checks if the next item fits into the stock
        self.space_available_in_stock_condition = Condition(
            sensor=self.next_item_fits_in_storage_sensor,
            activator=ThresholdActivator(
                thresholdValue=0,
                isMinimum=True)
        )
        # Only go to resource if there is space available in stock
        self.go_to_resource_node_behaviour.add_precondition(
            precondition=self.space_available_in_stock_condition
        )
        # Only ather if there is space available in stock
        self.gather_ingredients_behaviour.add_precondition(
            precondition=self.space_available_in_stock_condition
        )
        self.gather_ingredients_behaviour.add_precondition(
            precondition=Negation(self.has_all_ingredients_condition)
        )
        # TODO: Add a proper effect
        self.gather_ingredients_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.next_item_fits_in_storage_sensor.name,
                indicator=-1.0,
                sensor_type=float

            )
        )

    def init_go_to_resource_behaviour(self, agent, proximity):
        ################ Going to resource #########################
        self.go_to_resource_node_behaviour = GoToResourceBehaviour(
            agent=agent,
            name="go_to_resource_for_hoarding",
            plannerPrefix=self.get_manager_prefix(),
            product_provider_method=self._product_provider.get_required_ingredients_for_hoarding
        )
        self.resource_destination_sensor_hoarding = DestinationDistanceSensor(
            name='resource_destination_sensor_hoarding',
            agent_name=agent._agent_name,
            behaviour_name=self.go_to_resource_node_behaviour._name)
        self.at_resource_node_condition = Condition(
            sensor=self.resource_destination_sensor_hoarding,
            activator=ThresholdActivator(
                thresholdValue=proximity,
                isMinimum=False))  # highest activation if the value is below threshold
        # Only go to resource node if we aren't already there
        self.go_to_resource_node_behaviour.add_precondition(
            precondition=Negation(self.at_resource_node_condition))

        self.go_to_resource_node_behaviour.add_precondition(
            precondition=Negation(self.has_all_ingredients_condition)
        )
        # Going to resource has the effect of decreasing the distance to go there
        self.go_to_resource_node_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.resource_destination_sensor_hoarding.name,
                indicator=-1.0,
                sensor_type=float

            )
        )
    def init_product_sensors(self, agent):
        self.has_all_ingredients_sensor = ProductSensor(
            name="has_all_ingredients_for_hoarding_sensor",
            agent_name=agent._agent_name,
            product_provider_method=self._product_provider.get_required_ingredients_for_hoarding)
        self.has_all_ingredients_condition = Condition(
            sensor=self.has_all_ingredients_sensor,
            activator=AmountInListActivator(
                amount=0
            ))

        self.has_all_finished_products_sensor = ProductSensor(
            name="has_all_finished_products_for_hoarding_sensor",
            agent_name=agent._agent_name,
            product_provider_method=self._product_provider.get_required_finished_products_for_hoarding
        )
        self.has_all_finished_products_condition = Condition(
            sensor=self.has_all_finished_products_sensor,
            activator=AmountInListActivator(
                amount=0
            ))
        self.has_all_products_condition = Conjunction(
            self.has_all_ingredients_condition, self.has_all_finished_products_condition
        )

    def start(self):
        # TODO: This should be done at other places (e.g. when finishing gathering,..)

        stock = self._product_provider.calculate_desired_ingredient_stock()
        item_to_focus = max(stock, key=stock.get)
        self._product_provider.start_hoarding(item_to_focus)

        rospy.logerr("Focusing on %s", item_to_focus)
        super(HoardingNetworkBehaviour, self).start()

    def init_assembly_behaviour(self, agent):
        self.assemble_product_behaviour = AssembleProductBehaviour(
            name="assemble_product_for_hoarding_behaviour",
            agent_name=agent._agent_name,
            plannerPrefix=self.get_manager_prefix(),
            product_providermethod=self._product_provider.get_required_finished_products_for_hoarding
        )
        self.assemble_product_behaviour.add_precondition(
            precondition=Negation(self.space_available_in_stock_condition)
        )
        self.assemble_product_behaviour.add_precondition(
            precondition=self.at_workshop_condition
        )
        self.add_effect(
            effect=Effect(
                sensor_name=self.has_all_finished_products_sensor.name,
                indicator=1.0,
                sensor_type=bool
            )
        )

    def init_go_to_workshop_behaviour(self, agent, proximity):
        # go to destination
        self.go_to_workshop_behaviour = GoToWorkshopBehaviour(
            name="go_to_workshop_behaviour_for_hoarding",
            plannerPrefix=self.get_manager_prefix(),
            agent=agent,
            agent_name=agent._agent_name,
            topic="/workshop",
            product_providermethod = self._product_provider.get_required_finished_products_for_hoarding
        ) # TODO Need to figure out how and where to decide what to build

        # Only go to workshop if the stock is full and we need to make space
        self.go_to_workshop_behaviour.add_precondition(
            precondition=Negation(self.space_available_in_stock_condition)
        )

        self.workshop_destination_sensor = DestinationDistanceSensor(
            name='workshop_destination_sensor_for_hoarding',
            agent_name=agent._agent_name,
            behaviour_name=self.go_to_workshop_behaviour._name)

        self.go_to_workshop_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.workshop_destination_sensor.name,
                indicator=-1.0,
                sensor_type=float
            )
        )

        self.at_workshop_condition = Condition(
            sensor=self.workshop_destination_sensor,
            activator=ThresholdActivator(
                thresholdValue=proximity,
                isMinimum=False))  # highest activation if the value is below threshold

        self.go_to_workshop_behaviour.add_precondition(
            precondition=Negation(self.at_workshop_condition)
        )
