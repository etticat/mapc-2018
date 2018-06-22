from agent_knowledge.movement import MovementKnowledgebase
from behaviour_components.activators import ThresholdActivator, BooleanActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition, Negation
from behaviour_components.network_behavior import NetworkBehaviour
from behaviours.gather import ChooseIngredientBehaviour
from behaviours.job import GoToResourceBehaviour, GatherBehaviour
from provider.product_provider import ProductProvider
from sensor.agent import StorageFitsMoreItemsSensor
from sensor.job import ProductSensor
from sensor.movement import DestinationDistanceSensor


class GatheringNetworkBehaviour(NetworkBehaviour):

    def __init__(self, agent, name, msg, **kwargs):

        proximity = msg.proximity
        self._product_provider = ProductProvider(
            agent_name=agent._agent_name)

        self._movement_knowledge = MovementKnowledgebase()
        self._agent_name= agent._agent_name

        super(GatheringNetworkBehaviour, self).__init__(name, **kwargs)


        # Sensor to check how much space there is left after gaining the next intended item
        self.storage_fits_more_items_sensor = StorageFitsMoreItemsSensor(
            name="storage_fits_more_items_sensor",
            agent_name=agent._agent_name
        )
        # Checks if the next item fits into the stock
        self.next_item_fits_in_storage_condition = Condition(
            sensor=self.storage_fits_more_items_sensor,
            activator=BooleanActivator(desiredValue=True)
        )

        # self.init_product_sensor(agent)

        # self.init_choose_ingredient_behaviour(agent, proximity)
        # self.init_go_to_resource_behaviour(agent, proximity)
        # self.init_gather_behaviour(agent)


    def init_gather_behaviour(self, agent):
        ############### Gathering ##########################
        self.gather_ingredients_behaviour = GatherBehaviour(
            name="gather_for_hoarding_behaviour",
            agent_name=agent._agent_name,
            plannerPrefix=self.get_manager_prefix(),
            behaviour_name=self.go_to_resource_node_behaviour._name
        )
        # Only gather if we are at the intended resource node
        self.gather_ingredients_behaviour.add_precondition(
            precondition=self.at_resource_node_condition)

        # only gather if we have chosen an item to gather
        self.gather_ingredients_behaviour.add_precondition(
            precondition=Negation(self.has_all_ingredients_condition)
        )

        self.gather_ingredients_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.storage_fits_more_items_sensor.name,
                indicator=-1.0,
                sensor_type=bool

            )
        )

    def init_go_to_resource_behaviour(self, agent, proximity):
        ################ Going to resource #########################
        self.go_to_resource_node_behaviour = GoToResourceBehaviour(
            agent=agent,
            name="go_to_resource_node_behaviour",
            plannerPrefix=self.get_manager_prefix()
        )
        self.resource_destination_sensor_hoarding = DestinationDistanceSensor(
            name='resource_destination_sensor',
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
        # only go to resource node if we have chosen an item to gather
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
    def init_product_sensor(self, agent):
        self.number_of_products_to_gather_sensor = ProductSensor(
            name="number_of_products_to_gather_sensor",
            agent_name=agent._agent_name)
        self.has_all_ingredients_condition = Condition(
            sensor=self.number_of_products_to_gather_sensor,
            activator=ThresholdActivator(
                thresholdValue=0,
                isMinimum=False
            ))

    def stop(self):
        """
        Stoping all gather goals when leaving network
        :return:
        """
        self._product_provider.stop_gathering()
        # self._movement_knowledge.stop_movement(self._agent_name, self.go_to_resource_node_behaviour.name)
        super(GatheringNetworkBehaviour, self).stop()

    def init_choose_ingredient_behaviour(self, agent, proximity):
        self.choose_ingredient_behaviour = ChooseIngredientBehaviour(
            name="choose_ingredient_behaviour",
            agent_name=agent._agent_name,
            plannerPrefix=self.get_manager_prefix()
        )

        # only chose an item if we currently don't have a goal
        self.choose_ingredient_behaviour.add_precondition(
            precondition=self.has_all_ingredients_condition
        )

        # Chosing an ingredient has the effect, that we have more ingredients to gather
        self.choose_ingredient_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.number_of_products_to_gather_sensor.name,
                indicator=1.0,
                sensor_type=bool
            )
        )

