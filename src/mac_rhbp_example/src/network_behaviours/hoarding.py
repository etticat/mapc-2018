import rospy
from mac_ros_bridge.msg import Agent

from behaviour_components.activators import ThresholdActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition, Negation
from behaviour_components.goals import GoalBase
from behaviour_components.network_behavior import NetworkBehaviour
from behaviour_components.sensors import Sensor
from behaviours.job import GoToResourceForHoardingBehaviour, GatherForHoardingBehaviour
from common_utils.agent_utils import AgentUtils
from sensor.movement import DestinationDistanceSensor


class StockStorageAvailableSensor(Sensor):

    def __init__(self,agent_name, **kwargs):
        super(StockStorageAvailableSensor, self).__init__(**kwargs)

        self.free_load = 0
        rospy.Subscriber(AgentUtils.get_bridge_topic_agent(agent_name=agent_name), Agent, callback=self.callback_agent)

    def callback_agent(self, msg):
        """

        :param self:
        :param msg:
        :type msg: Agent
        :return:
        """

        volume_of_next_gathered_item = 5
        rospy.logerr("free load: %s", str(msg.load_max - msg.load - volume_of_next_gathered_item))
        self.update(msg.load_max - msg.load - volume_of_next_gathered_item)


class HoardingNetwork(NetworkBehaviour):

    def __init__(self, agent, name, msg, **kwargs):

        proximity = msg.proximity

        super(HoardingNetwork, self).__init__(name, **kwargs)


        ################ Going to resource #########################


        self.go_to_resource_node_behaviour = GoToResourceForHoardingBehaviour(
            agent=agent,
            name="go_to_resource_for_hoarding",
            plannerPrefix=self.get_manager_prefix())

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

        # Going to resource has the effect of decreasing the distance to go there
        self.go_to_resource_node_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.resource_destination_sensor_hoarding.name,
                indicator=-1.0,
                sensor_type=float

            )
        )

        ############### Gathering ##########################
        self.gather_ingredients_behaviour = GatherForHoardingBehaviour(
            name="gather_for_hoarding_behaviour",
            agent_name=agent._agent_name,
            plannerPrefix=self.get_manager_prefix())

        # Only gather if we are at the intended resource node
        self.gather_ingredients_behaviour.add_precondition(
            precondition=self.at_resource_node_condition)


        # Sensor to check how much space there is left after gaining the next intended item
        self.next_item_fits_in_storage_sensor  = StockStorageAvailableSensor(
            name="space_available_in_stock_sensor",
            agent_name=agent._agent_name
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



        # TODO: Add a proper effect
        self.gather_ingredients_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.next_item_fits_in_storage_sensor.name,
                indicator=-1.0,
                sensor_type=float

            )
        )  # TODO: this will be changed to int

        self._job_performance_goal = GoalBase(
            name='fill_up_stock',
            permanent=True,
            plannerPrefix=self.get_manager_prefix(),
            conditions=[Negation(self.space_available_in_stock_condition)])

