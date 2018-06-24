import rospy
from mac_ros_bridge.msg import Agent
from mapc_rhbp_ettlinger.msg import AssembleBid

from common_utils import rhbp_logging
from common_utils.agent_utils import AgentUtils
from provider.distance_provider import DistanceProvider
from provider.product_provider import ProductProvider

ettilog = rhbp_logging.LogManager(logger_name=rhbp_logging.LOGGER_DEFAULT_NAME + '.decisions.assembly_bid')

class ShouldBidForAssembly(object):

    WEIGHT_LOAD = 30
    WEIGHT_INGREDIENT_LOAD = 100
    WEIGHT_STEPS = -3 # In production this should be way smaller (Because close ones should be preferred)

    ACTIVATION_THRESHOLD = 0

    def __init__(self, agent_name, role):

        self._agent_name = agent_name
        self._product_provider = ProductProvider(agent_name=agent_name)
        self.distance_provider = DistanceProvider()

        self._sub_ref = rospy.Subscriber(AgentUtils.get_bridge_topic_agent(agent_name), Agent, self._callback_agent)

        self.max_load = None
        self.load = None
        self.load_ingredients = None
        self.load_finished_products = None
        self.pos = None
        self.role = role

        self.initialized = False

    def _callback_agent(self, agent):
        """

        :param agent:
        :type agent: Agent
        :return:
        """

        self.max_load = agent.load_max
        self.load = agent.load
        self.load_ingredients = self._product_provider.calculate_total_volume(self._product_provider.get_base_ingredients_in_stock())
        self.load_finished_products = self._product_provider.calculate_total_volume(self._product_provider.get_finished_products_in_stock())
        self.items = {}
        self.pos = agent.pos

        self.initialized = True

    def choose(self, request):

        if self.initialized == False:
            ettilog.loginfo("ShouldBidForAssembly(%s):: not initialized", self._agent_name)
            return
        ingredient_fullness = float(self.load_ingredients) / self.max_load
        general_fulness = float(self.load) / self.max_load

        steps_to_destination = self.distance_provider.calculate_steps(self.pos, request.destination)



        activation = ingredient_fullness * ShouldBidForAssembly.WEIGHT_INGREDIENT_LOAD \
                     + general_fulness * ShouldBidForAssembly.WEIGHT_LOAD \
                     + steps_to_destination * ShouldBidForAssembly.WEIGHT_STEPS

        ettilog.loginfo("ShouldBidForAssembly:: Choosing to bid ....")
        ettilog.loginfo("ShouldBidForAssembly:: ingredient_fullness (%f) * ShouldBidForAssembly.WEIGHT_INGREDIENT_LOAD(%f) = %f",
                       ingredient_fullness, ShouldBidForAssembly.WEIGHT_INGREDIENT_LOAD, ingredient_fullness * ShouldBidForAssembly.WEIGHT_INGREDIENT_LOAD)
        ettilog.loginfo("ShouldBidForAssembly:: general_fulness (%f) * ShouldBidForAssembly.WEIGHT_LOAD (%f) = %f",
                       general_fulness, ShouldBidForAssembly.WEIGHT_LOAD, general_fulness * ShouldBidForAssembly.WEIGHT_LOAD
                       )
        ettilog.loginfo("ShouldBidForAssembly:: steps_to_destination (%f) * ShouldBidForAssembly.WEIGHT_STEPS (%f) = %f",
                       steps_to_destination, ShouldBidForAssembly.WEIGHT_STEPS, steps_to_destination * ShouldBidForAssembly.WEIGHT_STEPS)
        ettilog.loginfo("ShouldBidForAssembly:: activation (%f) > ShouldBidForAssembly.ACTIVATION_THRESHOLD (%f) = %s",
                       activation, ShouldBidForAssembly.ACTIVATION_THRESHOLD, str(activation > ShouldBidForAssembly.ACTIVATION_THRESHOLD))

        # TODO: Oportunity cost (certain roles could better do something else?

        if activation > ShouldBidForAssembly.ACTIVATION_THRESHOLD:
            return AssembleBid(
                id=request.id,
                bid = activation,
                agent_name = self._agent_name,
                items = self._product_provider.get_items(),
                role = self.role,
                request = request,
                expected_steps=steps_to_destination
            )
        else:
            return None
