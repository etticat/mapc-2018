import rospy
from mac_ros_bridge.msg import Position, Agent
from mapc_rhbp_ettlinger.msg import AssembleBid

from common_utils.agent_utils import AgentUtils
from provider.product_provider import ProductProvider
from provider.step_provider import StepProvider


class ShouldBidForAssembly(object):

    WEIGHT_LOAD = 30
    WEIGHT_INGREDIENT_LOAD = 100
    WEIGHT_STEPS = -0

    ACTIVATION_THRESHOLD = 70

    def __init__(self, agent_name, role):

        self._agent_name = agent_name
        self._product_provider = ProductProvider(agent_name=agent_name)
        self.set_provider = StepProvider()

        self._sub_ref = rospy.Subscriber(AgentUtils.get_bridge_topic_agent(agent_name), Agent, self._callback_agent)

        self.max_load = None
        self.load = None
        self.load_ingredients = None
        self.load_finished_products = None
        self.speed = None
        self.transport = None
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
        self.speed = agent.speed
        self.transport = "road" # TODO
        self.items = {}
        self.pos = agent.pos

        self.initialized = True

    def choose(self, request):

        if self.initialized == False:
            rospy.logerr("ShouldBidForAssembly(%s):: not initialized", self._agent_name)
            return
        ingredient_fullness = float(self.load_ingredients) / self.max_load
        general_fulness = float(self.load) / self.max_load

        steps_to_destination = self.set_provider.calculate_steps(self.pos, request.destination, self.speed, self.transport)



        activation = ingredient_fullness * ShouldBidForAssembly.WEIGHT_INGREDIENT_LOAD \
                     + general_fulness * ShouldBidForAssembly.WEIGHT_LOAD \
                     + steps_to_destination * ShouldBidForAssembly.WEIGHT_STEPS

        # TODO: Oportunity cost (certain roles could better do something else?

        if activation > ShouldBidForAssembly.ACTIVATION_THRESHOLD:
            return AssembleBid(
                id=request.id,
                bid = activation,
                agent_name = self._agent_name,
                items = self._product_provider.get_items(), # TODO: Read from db
                role = self.role,
                request = request
            )
        else:
            return None
