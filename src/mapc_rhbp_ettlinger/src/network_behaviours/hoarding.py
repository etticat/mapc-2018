from behaviours.hoarding import StoreBehaviour
from network_behaviours.go_and_do import GoAndDoNetworkBehaviour
from provider.product_provider import ProductProvider


class HoardingNetworkBehaviour(GoAndDoNetworkBehaviour):
    """
    Network behaviour responsible for Bringing finished products to a storage
    """

    def __init__(self, agent_name, name, shared_components, **kwargs):
        super(HoardingNetworkBehaviour, self).__init__(
            agent_name=agent_name,
            shared_components=shared_components,
            name=name,
            use_name_for_movement=True,
            mechanism=shared_components.choose_hoarding_decision,
            **kwargs)

        self._product_provider = ProductProvider(agent_name=agent_name)

        self.init_do_behaviour(StoreBehaviour(
            name="store_behaviour",
            agent_name=agent_name,
            plannerPrefix=self.get_manager_prefix(),
            mechanism=shared_components.choose_hoarding_decision
        ))

    def stop(self):
        """
        When stopping the behaviour, make sure to reset all hoarding tasks and goals
        :return:
        """
        super(HoardingNetworkBehaviour, self).stop()
        self._shared_components.choose_hoarding_decision.reset_value()