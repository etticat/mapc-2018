from behaviours.hoarding import StoreBehaviour
from network_behaviours.go_and_do import GoAndDoNetworkBehaviour
from provider.product_provider import ProductProvider


class HoardingNetworkBehaviour(GoAndDoNetworkBehaviour):

    def __init__(self, agent_name, name, sensor_map, **kwargs):
        super(HoardingNetworkBehaviour, self).__init__(
            agent_name=agent_name,
            sensor_map=sensor_map,
            name=name,
            mechanism=sensor_map.choose_hoarding_mechanism,
            **kwargs)

        self._product_provider = ProductProvider(agent_name=agent_name)
        store_behaviour = StoreBehaviour(
            name="store_behaviour",
            agent_name=agent_name,
            plannerPrefix=self.get_manager_prefix()
        )
        self.init_do_behaviour(store_behaviour)