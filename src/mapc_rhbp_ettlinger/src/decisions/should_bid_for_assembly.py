from mac_ros_bridge.msg import Agent
from mapc_rhbp_ettlinger.msg import TaskBid

import rospy
from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from provider.agent_info_provider import AgentInfoProvider
from provider.distance_provider import DistanceProvider
from provider.product_provider import ProductProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.decisions.assembly_bid')


class ShouldBidForAssemblyDecision(object):
    """
    Decision object, that decisdes if agent should bid for assembly decision
    """

    WEIGHT_LOAD = 30
    WEIGHT_INGREDIENT_LOAD = 100
    WEIGHT_STEPS = -3 # In production this should be way smaller (Because close ones should be preferred)

    ACTIVATION_THRESHOLD = 0

    def __init__(self, agent_name, role):
        self._agent_name = agent_name
        self.role = role

        self._max_load = None
        self._load = None
        self._load_ingredients = None
        self._load_finished_products = None
        self._pos = None

        self._initialized = False

        self._init_config()

        self._agent_info_provider = AgentInfoProvider(agent_name=agent_name)
        self._product_provider = ProductProvider(agent_name=agent_name)
        self._distance_provider = DistanceProvider(agent_name=agent_name)

        self._sub_ref = rospy.Subscriber(AgentUtils.get_bridge_topic_agent(agent_name), Agent, self._callback_agent)


    def _init_config(self):
        ShouldBidForAssemblyDecision.WEIGHT_LOAD = rospy.get_param("~ShouldBidForAssembly.WEIGHT_LOAD",
                                                                   ShouldBidForAssemblyDecision.WEIGHT_LOAD)
        ShouldBidForAssemblyDecision.WEIGHT_INGREDIENT_LOAD = rospy.get_param("~ShouldBidForAssembly.WEIGHT_INGREDIENT_LOAD",
                                                                              ShouldBidForAssemblyDecision.WEIGHT_INGREDIENT_LOAD)
        ShouldBidForAssemblyDecision.WEIGHT_STEPS = rospy.get_param("~ShouldBidForAssembly.WEIGHT_STEPS",
                                                                    ShouldBidForAssemblyDecision.WEIGHT_STEPS)
        ShouldBidForAssemblyDecision.ACTIVATION_THRESHOLD = rospy.get_param("~ShouldBidForAssembly.ACTIVATION_THRESHOLD",
                                                                            ShouldBidForAssemblyDecision.ACTIVATION_THRESHOLD)

    def _callback_agent(self, agent):
        """
        Get current values from agent
        :param agent:
        :type agent: Agent
        :return:
        """

        self._max_load = agent.load_max
        self._load = agent.load
        self._load_ingredients = self._product_provider.calculate_total_volume_dict(
            self._product_provider.get_base_ingredients_in_stock())
        self._load_finished_products = self._product_provider.calculate_total_volume_dict(
            self._product_provider.get_finished_products_in_stock())
        self.items = {}
        self._pos = agent.pos

        self._initialized = True

    def generate_assembly_bid(self, request):
        """
        Decides if agent should bid for assembly and if it decides for yes, it generates an assembly bid
        :param request:
        :return:
        """

        if not self._initialized:
            # In case we haven't gotten the first agent callback already, return None
            ettilog.loginfo("ShouldBidForAssembly(%s):: not initialized", self._agent_name)
            return None

        ingredient_fullness = float(self._load_ingredients) / self._max_load
        general_fullness = float(self._load) / self._max_load
        steps_to_destination = self._distance_provider.calculate_steps(self._pos, request.destination)

        activation = ingredient_fullness * ShouldBidForAssemblyDecision.WEIGHT_INGREDIENT_LOAD \
                     + general_fullness * ShouldBidForAssemblyDecision.WEIGHT_LOAD \
                     + steps_to_destination * ShouldBidForAssemblyDecision.WEIGHT_STEPS

        if activation > ShouldBidForAssemblyDecision.ACTIVATION_THRESHOLD:
            return TaskBid(
                id=request.id,
                bid=activation,
                agent_name=self._agent_name,
                items=self._product_provider.get_item_list(),
                role=self.role,
                capacity=self._product_provider.load_free,
                skill=self._agent_info_provider.skill,
                speed=self._distance_provider.speed,
                finished_product_factor=self._product_provider.finished_product_load_factor(),
                request=request,
                expected_steps=steps_to_destination
            )
        else:
            return None
