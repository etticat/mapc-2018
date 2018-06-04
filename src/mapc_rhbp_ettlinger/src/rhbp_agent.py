#!/usr/bin/env python2

import rospy
from mac_ros_bridge.msg import RequestAction, GenericAction, SimStart, SimEnd, Bye, sys

from agent_knowledge.item import StockItemKnowledgebase
from agent_knowledge.resource import ResourceKnowledgebase
from common_utils.agent_utils import AgentUtils
from behaviour_components.activators import ThresholdActivator
from behaviour_components.condition_elements import Effect
from behaviour_components.conditions import Negation, Condition, Disjunction
from behaviour_components.managers import Manager
from network_behaviours.assemble import AssembleNetworkBehaviour
from network_behaviours.assist import AssistNetworkBehaviour
from network_behaviours.battery import BatteryChargingNetworkBehaviour
from network_behaviours.exploration import ExplorationNetworkBehaviour
from network_behaviours.gather import GatheringNetworkBehaviour
from network_behaviours.job_execution import JobExecutionNetworkBehaviour
from provider.product_provider import ProductProvider


class RhbpAgent:
    """
    Main class of an agent, taking care of the main interaction with the mac_ros_bridge
    """

    def __init__(self, agent_name):
        """

        :param agent_name:
        :type agent_name:  str
        """
        self.facilityKnowledgebase = ResourceKnowledgebase()

        rospy.init_node('agent_node', anonymous=True, log_level=rospy.ERROR)

        # Take the name from the constructor parameter in case it was started from a development environment
        if agent_name != None:
            self._agent_name = agent_name
        else:
            self._agent_name = rospy.get_param('~agent_name', "agentA1")  # default for debugging 'agentA1'

        rospy.logerr("RhbpAgent:: Starting %s", self._agent_name)
        self._agent_topic_prefix = AgentUtils.get_bridge_topic_prefix(agent_name=self._agent_name)

        # ensure also max_parallel_behaviours during debugging
        self._manager = Manager(prefix=self._agent_name, max_parallel_behaviours=1)

        self._sim_started = False
        self._initialized = False

        # subscribe to MAC bridge core simulation topics
        rospy.Subscriber(self._agent_topic_prefix + "request_action", RequestAction, self._action_request_callback)

        rospy.Subscriber(self._agent_topic_prefix + "start", SimStart, self._sim_start_callback)

        rospy.Subscriber(self._agent_topic_prefix + "end", SimEnd, self._sim_end_callback)

        rospy.Subscriber(self._agent_topic_prefix + "bye", Bye, self._bye_callback)

        rospy.Subscriber(self._agent_topic_prefix + "generic_action", GenericAction, self._callback_generic_action)

        self._received_action_response = False

    def _sim_start_callback(self, msg):
        """
        here we could also evaluate the msg in order to initialize depending on the role etc.
        :param msg:  the message
        :type msg: SimStart
        """

        if not self._sim_started:  # init only once here

            self._sim_started = True
            rospy.loginfo(self._agent_name + " startet")

            # init only once, even when run restarts
            if not self._initialized:
                self.init_behaviour_network(msg)
                self.init_behaviour_network_connections()

            self._initialized = True

    def init_behaviour_network(self, msg):

        ######################## Battery Network Behaviour ########################
        self._battery_charging_network_behaviour = BatteryChargingNetworkBehaviour(
            name=self._agent_name + '/BatteryChargingNetwork',
            plannerPrefix=self._agent_name,
            agent=self,
            msg=msg,
            max_parallel_behaviours=1)


        ######################## Job Network Behaviour ########################
        self._job_performance_network = JobExecutionNetworkBehaviour(
            name=self._agent_name + '/JobPerformanceNetwork',
            plannerPrefix=self._agent_name,
            msg=msg,
            agent=self,
            max_parallel_behaviours=1)

        ######################## Gathering Network Behaviour ########################
        self._gathering_network = GatheringNetworkBehaviour(
            name=self._agent_name + '/GatheringNetwork',
            plannerPrefix=self._agent_name,
            msg=msg,
            agent=self,
            max_parallel_behaviours=1)

        ######################## Assembly Network Behaviour ########################
        self._assembly_network = AssembleNetworkBehaviour(
            name=self._agent_name + '/AssembleNetwork',
            plannerPrefix=self._agent_name,
            msg=msg,
            agent=self,
            max_parallel_behaviours=1)

        ######################## Assist Network Behaviour ########################
        self._assist_task_network = AssistNetworkBehaviour(
            agent_name=self._agent_name,
            name=self._agent_name + '/AssistNetwork',
            plannerPrefix=self._agent_name,
            msg = msg,
            max_parallel_behaviours=1)

        ######################## Exploration Network Behaviour ########################
        self._exploration_network = ExplorationNetworkBehaviour(
            name=self._agent_name + '/ExplorationNetwork',
            plannerPrefix=self._agent_name,
            agent=self,
            msg=msg,
            max_parallel_behaviours=1)

        ######################## Build Well Network Behaviour ########################
        # self._build_wells_network_behaviour = BuildWellNetworkBehaviour(
        #     name=self._agent_name + '/BuildWellsNetwork',
        #     plannerPrefix=self._agent_name,
        #     agent=self,
        #     msg=msg,
        #     max_parallel_behaviours=1)

        rospy.logerr("behaviour initialized")

    def init_behaviour_network_connections(self):

        ######################## Battery Network Behaviour ########################
        

        # Battery network has the effect of charging the battery
        # TODO: QQQ Is this needed?
        self._battery_charging_network_behaviour.add_effects_and_goals([(
            self._battery_charging_network_behaviour._charge_sensor,
            Effect(
                sensor_name=self._battery_charging_network_behaviour._charge_sensor.name,
                indicator=1.0,
                sensor_type=float))])
        

        # Only charge if charge is required
        self._battery_charging_network_behaviour.add_precondition(
            precondition=self._battery_charging_network_behaviour._require_charging_cond)


        ######################## Job Network Behaviour ########################

        # Only perform jobs if there is enough charge left
        self._job_performance_network.add_precondition(
            precondition=self._battery_charging_network_behaviour._enough_battery_cond)

        # Is done implicitly already through goal QQQ: Do I really need this?
        self._job_performance_network.add_precondition(
            precondition=self._job_performance_network.has_tasks_assigned_condition)

        # Only perform tasks when there is no assist required
        self._job_performance_network.add_precondition(
            precondition=Negation(self._assist_task_network.assist_assigned_condition))

        # TODO: Do I need these? QQQ
        # Everything seems to work well without it. With it I run into errors
        # undeclared predicate has_task used in domain definition
        # [ERROR] [1527777061.931515]: PLANNER ERROR: Planner exited with failure.. Generating PDDL log files for step 11
        # self._job_performance_network.add_effects_and_goals([(
        #     self._job_performance_network.has_tasks_assigned_sensor,
        #     Effect(
        #         sensor_name=self._job_performance_network.has_tasks_assigned_sensor.name,
        #         indicator=-1.0,
        #         sensor_type=bool
        #            )
        #
        # )])

        
        ######################## Gathering Network Behaviour ########################

        # Only gather when there is enough battery left
        self._gathering_network.add_precondition(
            precondition=self._battery_charging_network_behaviour._enough_battery_cond)

        # Only gather if there is enough storage left
        # TODO: This should be a linear activator or something
        # Currently the problem is when an agent almost fills up their stock with one item while still having place
        # for one item of a different kind, they may go to the oposite side of the map. This is quite inefficient
        self._gathering_network.add_precondition(
            precondition=self._gathering_network.next_item_fits_in_storage_condition
        )
        # Only gather when there is nothing to be assembled
        self._gathering_network.add_precondition(
            precondition=self._assembly_network.has_all_finished_products_condition)

        # Only gather if there is no task assigned
        # TODO: We might want to allow this in the case where the agent does not have all items for the task
        # I do not think this makes sense as only accepting tasks where the agent already has all items seems more
        # promising. Might want to reconsider this at a later stage when the job execution is more clear
        self._gathering_network.add_precondition(
            precondition=Negation(self._job_performance_network.has_tasks_assigned_condition))

        # Only perform gather when there is no assist required
        self._gathering_network.add_precondition(
            precondition=Negation(self._assist_task_network.assist_assigned_condition))

        # Only gather if all resources are discovered
        self._gathering_network.add_precondition(
            precondition=self._exploration_network.all_resources_discovered_condition)
        

        ######################## Assembly Network Behaviour ########################
        # Only assemble if there is enough battery left
        self._assembly_network.add_precondition(
            precondition=self._battery_charging_network_behaviour._enough_battery_cond)

        # We should not assemble before the trunk is full. (except when we are already assembling)
        self._assembly_network.add_precondition(
            precondition=Disjunction(
                # Start it when the storage is full
                Negation(self._gathering_network.next_item_fits_in_storage_condition),
                # But keep it active until all products are assembled
                Negation(self._assembly_network.has_all_finished_products_condition)
            )
        )

        # Only assist when an assist_task is assigned
        # TODO: QQQ This should not be required. as the network should not be executed anyway when the goal is fulfilled.
        self._assist_task_network.add_precondition(
            precondition=self._assist_task_network.assist_assigned_condition)

        # Only assemble, when there is no assist required
        self._assembly_network.add_precondition(
            precondition=Negation(self._assist_task_network.assist_assigned_condition))



        ######################## Exploration Network Behaviour ########################

        # Only do shop exploration when enough battery left
        self._exploration_network.add_precondition(
            precondition=self._battery_charging_network_behaviour._enough_battery_cond)

        # Only explore when there is not task assigned
        self._exploration_network.add_precondition(
            precondition=Negation(self._job_performance_network.has_tasks_assigned_condition))

        # Only explore when not all resource nodes are discovered
        # TODO: Maybe it makes sense that some agents continue to explore
        self._exploration_network.add_precondition(
            precondition=Negation(self._exploration_network.all_resources_discovered_condition)
        )

        

        # The overall goal is to gain score (For now to have wells)
        # TODO: Goal should be high well number. Will be changed once wells are implemented.
        # QQQ Does this make sense?

        # TODO: Do I need these #33-1
        # Seems to work without. With them I get errors but it continues to run normally
        # self._job_performance_goal = GoalBase(
        #     name='score_goal',
        #     permanent=True,
        #     plannerPrefix=self._agent_name,
        #     conditions=[Negation(self._job_performance_network.has_tasks__assigned_condition)])

        rospy.logerr("behaviour connections initialized")
    def _callback_generic_action(self, msg):
        """
        ROS callback for generic actions
        :param msg: ros message
        :type msg: GenericAction
        """
        self._received_action_response = True

    def _sim_end_callback(self, msg):
        """
        :param msg:  the message
        :type msg: SimEnd
        """
        rospy.loginfo("SimEnd:" + str(msg))
        self._sim_started = False

    def _bye_callback(self, msg):
        """
        :param msg:  the message
        :type msg: Bye
        """
        rospy.loginfo("Bye:" + str(msg))

    def _action_request_callback(self, msg):
        """
        here we just trigger the decision-making and plannig
        :param msg: the message
        :type msg: RequestAction
        :return:
        """

        start_time = rospy.get_rostime()
        rospy.logdebug("RhbpAgent::callback %s", str(msg))

        self.agent_info = msg.agent

        for resource in msg.resources:
            self.facilityKnowledgebase.add_new_resource(resource)

        self._received_action_response = False

        # for i in range(len(self._gathering_network._preconditions)):
        #     rospy.logerr("precondition (%s): %s",self._gathering_network._preconditions[i]._name, str(self._gathering_network._get_satisfactions()[i]))
        #
        # rospy.logerr("------")

        stock_item_knowledgebase = StockItemKnowledgebase()
        all = stock_item_knowledgebase.get_total_stock_and_goals()

        for item in all.keys():
            rospy.logerr("%s: %s/%s",item, str(all[item]["stock"]), str(all[item]["goal"]))

        start_time = rospy.get_rostime()
        # wait for generic action response (send by any behaviour)
        while not self._received_action_response:
            self._manager.step() # selected behaviours eventually trigger action
            # Recharge if decision-making-time > 3.9 seconds
            if (rospy.get_rostime() - start_time).to_sec() > 3.9:
                rospy.logerr(
                    "%s: Decision-making duration exceeded 3.9 seconds. Recharging.",
                    self._agent_name)
                #
                # pub_generic_action = rospy.Publisher(
                #     self._agent_topic_prefix + 'generic_action',
                #     GenericAction, queue_size=10)
                # action_generic(publisher=pub_generic_action,
                #                action_type=Action.RECHARGE)
                break

        duration = rospy.get_rostime() - start_time
        rospy.logerr("%s: Decision-making duration %f", self._agent_name, duration.to_sec())





if __name__ == '__main__':

    agent_name = None

    if len(sys.argv) == 3 and sys.argv[1] == "--agent-name":
        agent_name = sys.argv[2]

    try:
        rhbp_agent = RhbpAgent(agent_name)

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
