from mac_ros_bridge.msg import Position

from agent_knowledge.facilities import FacilityKnowledgebase
from agent_knowledge.tasks import TaskKnowledgebase
from behaviour_components.activators import BooleanActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition, Negation
from behaviour_components.goals import GoalBase
from behaviour_components.network_behavior import NetworkBehaviour
from behaviours.movement import GotoLocationBehaviour
from rhbp_utils.knowledge_sensors import KnowledgeSensor


class BuildWellNetworkBehaviour(NetworkBehaviour):

    def __init__(self, agent, name, **kwargs):

        super(BuildWellNetworkBehaviour, self).__init__(name, **kwargs)

        #TODO: This is just a temporary sensor that is always false/0
        self.number_of_wells_sensor = KnowledgeSensor(
            name='well_built',
            pattern=("well_built", "true"))

        # TODO: Change this to a count condition when wells are implemented
        self.has_well = Condition(
            sensor=self.number_of_wells_sensor,
            activator=BooleanActivator(
                desiredValue=True))

        #TODO: This is just a temporary sensor that is always false/0
        self.money_sensor = KnowledgeSensor(
            name='money_sensor',
            pattern=("have enough money", "true"))

        # TODO: Change this to count money condition
        self.has_enough_money_for_next_well = Condition(
            sensor=self.money_sensor,
            activator=BooleanActivator(
                desiredValue=True))

        self.add_precondition(self.has_enough_money_for_next_well)

        self.add_effect(
            effect=Effect(
                sensor_name=self.number_of_wells_sensor.name,
                indicator=1.0,
                sensor_type=bool))


