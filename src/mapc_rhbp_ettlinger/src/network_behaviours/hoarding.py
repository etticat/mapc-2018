from behaviours.hoarding import StoreBehaviour
from network_behaviours.go_and_do import GoAndDoNetworkBehaviour
from provider.product_provider import ProductProvider


class HoardingNetworkBehaviour(GoAndDoNetworkBehaviour):
    """
    Network behaviour responsible for Bringing finished products to a storage
    """

    def __init__(self, agent_name, name, global_rhbp_components, **kwargs):
        super(HoardingNetworkBehaviour, self).__init__(
            agent_name=agent_name,
            global_rhbp_components=global_rhbp_components,
            name=name,
            mechanism=global_rhbp_components.choose_hoarding_mechanism,
            **kwargs)

        self._product_provider = ProductProvider(agent_name=agent_name)

        self.init_do_behaviour(StoreBehaviour(
            name="store_behaviour",
            agent_name=agent_name,
            plannerPrefix=self.get_manager_prefix(),
            mechanism=global_rhbp_components.choose_hoarding_mechanism
        ))

    def stop(self):
        """
        When stoping the behaviour, make sure to reset all hoarding tasks and goals
        :return:
        """
        super(HoardingNetworkBehaviour, self).stop()
        self._global_rhbp_components.choose_hoarding_mechanism.reset_value()