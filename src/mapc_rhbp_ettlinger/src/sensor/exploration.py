from mac_ros_bridge.msg import SimEnd

import rospy

from behaviour_components.sensors import Sensor
from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from decisions.choose_well_to_build import OldestCellAgeDecision, DiscoverProgressDecision
from provider.facility_provider import FacilityProvider
from provider.product_provider import ProductProvider
from provider.self_organisation_provider import SelfOrganisationProvider
from rhbp_selforga.gradientsensor import GradientSensor, SENSOR

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.sensors.exploration')


class ResourceDiscoveryProgressSensor(Sensor):
    """
    Calulates the percentage of gatherable items, where a resouce node is already found
    """

    def __init__(self, agent_name, optional=False, name=None, initial_value=0.0):
        super(ResourceDiscoveryProgressSensor, self).__init__(name=name, optional=optional, initial_value=initial_value)

        self._facility_provider = FacilityProvider(agent_name=agent_name)
        self._product_provider = ProductProvider(agent_name=agent_name)

        # Reset sensor when simulation ends
        rospy.Subscriber(AgentUtils.get_bridge_topic(agent_name=agent_name, postfix="end"), SimEnd, self.reset_sensor)


    def sync(self):
        # If the value reached 1.0, it will never go down. No need to calculate it anymore
        if self._value == 1.0:
            return super(ResourceDiscoveryProgressSensor, self).sync()

        resources = self._facility_provider.get_resources()
        base_ingredients = self._product_provider.get_gatherable_ingredients().keys()
        number_of_base_ingredients = len(base_ingredients)

        for resource in resources.values():
            if resource.item.name in base_ingredients:
                base_ingredients.remove(resource.item.name)

        number_of_discovered_ingredients = number_of_base_ingredients - len(base_ingredients)
        if number_of_base_ingredients > 0:
            result = float(number_of_discovered_ingredients) / number_of_base_ingredients
            ettilog.loginfo("%s:: Discovery progress: %s", self.name, str(result))
        else:
            result = 0

        self.update(result)
        return super(ResourceDiscoveryProgressSensor, self).sync()

    def reset_sensor(self, sim_end):
        """
        Reset the progress when simulation ends
        :param sim_end:
        :return:
        """
        self._value = 0


class DiscoveryProgressSensor(GradientSensor):
    """
    Calculates the percentage of the map that has been discovered
    """

    def __init__(self, agent_name, name, thread=False, time=5, initial_value=None, sensor_type=SENSOR.VALUE):
        self._self_organisation_provider = SelfOrganisationProvider(agent_name=agent_name)

        self._discovery_progress_decision = DiscoverProgressDecision(self._self_organisation_provider.so_buffer, agent_name=agent_name)

        super(DiscoveryProgressSensor, self).__init__(name=name, mechanism=self._discovery_progress_decision, thread=thread, time=time,
                                                      initial_value=initial_value, sensor_type=sensor_type)


class OldestCellLastSeenSensor(GradientSensor):
    """
    Calculates maximum number of steps a cell on the map has been visited
    """

    def __init__(self, agent_name, name, initial_value, thread=False, time=5, sensor_type=SENSOR.VALUE):
        self.self_organisation_provider = SelfOrganisationProvider(agent_name=agent_name)

        self.discovery_progress_decision = OldestCellAgeDecision(self.self_organisation_provider.so_buffer,
                                                                 init_value=initial_value, agent_name=agent_name)

        super(OldestCellLastSeenSensor, self).__init__(name, self.discovery_progress_decision, thread, time,
                                                       initial_value, sensor_type)
