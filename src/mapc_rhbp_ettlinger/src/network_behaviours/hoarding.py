from agent_knowledge.local_knowledge_sensors import LocalKnowledgeSensor
from agent_knowledge.task import TaskKnowledgeBase
from behaviour_components.activators import BooleanActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Condition, Negation
from behaviours.hoarding import ChooseStorageBehaviour, StoreBehaviour
from network_behaviours.go_and_do import GoAndDoNetworkBehaviour
from provider.product_provider import ProductProvider
from rhbp_utils.knowledge_sensors import KnowledgeSensor


class HoardingNetworkBehaviour(GoAndDoNetworkBehaviour):

    def __init__(self, agent_name, name, sensor_map, **kwargs):
        super(HoardingNetworkBehaviour, self).__init__(
            agent_name=agent_name,
            sensor_map=sensor_map,
            name=name,
            task_type=TaskKnowledgeBase.TYPE_HOARDING,
            **kwargs)

        self._product_provider = ProductProvider(agent_name=agent_name)
        self._task_knowledge_base = TaskKnowledgeBase()
        store_behaviour = StoreBehaviour(
            name="store_behaviour",
            agent_name=agent_name,
            plannerPrefix=self.get_manager_prefix()
        )
        self.init_do_behaviour(store_behaviour)
        self.init_choose_storage_behaviour()
        self._do_behaviour.add_precondition(self.has_hoarding_task_cond)
        self._go_behaviour.add_precondition(self.has_hoarding_task_cond)


    def init_choose_storage_behaviour(self):
        self.choose_storage_behaviour = ChooseStorageBehaviour(
            name="choose_storage_behaviour",
            agent_name=self._agent_name,
            plannerPrefix=self.get_manager_prefix()
        )
        self.has_hoarding_task_sensor = LocalKnowledgeSensor(
            name="has_hoarding_task_sensor",
            pattern=TaskKnowledgeBase.generate_tuple(
                agent_name=self._agent_name,
                type=TaskKnowledgeBase.TYPE_HOARDING
            ),
            knowledge_base_name=TaskKnowledgeBase.KNOWLEDGE_BASE_NAME
        )
        self.has_hoarding_task_cond = Condition(
            sensor=self.has_hoarding_task_sensor,
            activator=BooleanActivator(desiredValue=True)
        )

        # only chose an item if we currently don't have a goal
        self.choose_storage_behaviour.add_precondition(
            precondition=Negation(self.has_hoarding_task_cond)
        )

        # Chosing an ingredient has the effect, that we have more ingredients to gather
        self.choose_storage_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.has_hoarding_task_sensor.name,
                indicator=1.0,
                sensor_type=bool
            )
        )

    def stop(self):
        super(HoardingNetworkBehaviour, self).stop()
        # Deleting task and cleaning up goal KB
        self._task_knowledge_base.finish_task(type=TaskKnowledgeBase.TYPE_HOARDING, agent_name=self._agent_name)
        # self._product_provider.stop_hoarding()
        # TODO: Maybe save this in DB?