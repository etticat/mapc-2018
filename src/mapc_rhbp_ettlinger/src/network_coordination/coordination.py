#!/usr/bin/env python2
import rospy

from agent_knowledge.assemble_task import AssembleKnowledgebase
from behaviour_components.activators import BooleanActivator
from behaviour_components.behaviours import BehaviourBase
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Negation, Condition
from behaviour_components.network_behavior import NetworkBehaviour
from network_coordination.assemble_contractor import AssembleContractor
from network_coordination.assemble_manager import AssembleManager
from network_coordination.job_contractor import JobContractor
from rhbp_utils.knowledge_sensors import KnowledgeSensor


class CoordinationNetworkBehaviour(NetworkBehaviour):
    def __init__(self, agent, name, msg, **kwargs):

        super(CoordinationNetworkBehaviour, self).__init__(name, **kwargs)


        self.init_product_sensor(agent)
        self.init_choose_finished_products_behaviour(agent, msg.role)

        self._agent_name = agent._agent_name
        self._agent = agent


    def init_choose_finished_products_behaviour(self, agent, role):
        self.assembly_manager_behaviour = AssemblyManagerBehaviour(
            name="assembly_manager_behaviour",
            agent_name=agent._agent_name,
            plannerPrefix=self.get_manager_prefix(),
            role=role.name
        )
        # TODO: Doesnt work in parallel somehow
        # self.assembly_contractor_behaviour = AssemblyContractorBehaviour(
        #     name="assembly_contractor_behaviour",
        #     agent_name=agent._agent_name,
        #     plannerPrefix=self.get_manager_prefix(),
        #     role=role.name
        # )

        # only chose an item if we currently don't have a goal
        # self.assembly_contractor_behaviour.add_precondition(
        #     precondition=Negation(self.has_assemble_task_assigned_cond)
        # )

        # only start manager if no task assigned
        self.assembly_manager_behaviour.add_precondition(
            precondition=Negation(self.has_assemble_task_assigned_cond)
        )

        # Chosing an ingredient has the effect, that we have more ingredients to gather
        # self.assembly_contractor_behaviour.add_effect(
        #     effect=Effect(
        #         sensor_name=self.assemble_organized_sensor.name,
        #         indicator=1.0,
        #         sensor_type=bool
        #     )
        # )
        # Chosing an ingredient has the effect, that we have more ingredients to gather
        self.assembly_manager_behaviour.add_effect(
            effect=Effect(
                sensor_name=self.assemble_organized_sensor.name,
                indicator=1.0,
                sensor_type=bool
            )
        )
        # TODO: Manager precondition: trunk full
        # TODO: Contractor precondition: trunk half full

    def init_product_sensor(self, agent):
        self.assemble_organized_sensor = KnowledgeSensor(
            name="assemble_organized_sensor",
            pattern=AssembleKnowledgebase.generate_tuple(
                agent_name=agent._agent_name,
                active=True))

        self.has_assemble_task_assigned_cond = Condition(
            sensor=self.assemble_organized_sensor,
            activator=BooleanActivator(
                desiredValue=True
            ))


class AssemblyManagerBehaviour(BehaviourBase):

    def __init__(self, agent_name, role, **kwargs):
        super(AssemblyManagerBehaviour, self).__init__(requires_execution_steps=True, **kwargs)

        self._agent_name = agent_name
        # self._assemble_manager = AssembleManager(agent_name=agent_name, role=role)
        # For now -> lets do this in a seperate class

        self.assemble_contractor = AssembleContractor( # TODO: This should be in other behaviour. Should work in parallel
            agent_name=self._agent_name,
            role=role)
        self.assemble_contractor = JobContractor( # TODO: This should be in other behaviour. Should work in parallel
            agent_name=self._agent_name)
        self.assemble_contractor.enabled = True

    def do_step(self):
        # self._assemble_manager.request_assist()
        pass

class AssemblyContractorBehaviour(BehaviourBase):

    def __init__(self, agent_name, role, **kwargs):
        super(AssemblyContractorBehaviour, self).__init__(requires_execution_steps=False, **kwargs)

        self._agent_name = agent_name
        self._assemble_manager = AssembleManager(agent_name=agent_name, role=role)

        self.assemble_contractor = AssembleContractor(
            agent_name=self._agent_name,
            role=role)
        self.assemble_contractor.enabled = True # TODO: This behaviour never gets started: TODO: Find out why
        # For now we just always keep it active

    def start(self):
        self.assemble_contractor.enabled = True
        super(AssemblyContractorBehaviour, self).start()

    def stop(self):
        # self.assemble_contractor.enabled = False
        super(AssemblyContractorBehaviour, self).stop()

    def do_step(self):
        rospy.logerr("AssemblyContractorBehaviour(%s):: running", self._agent_name)
