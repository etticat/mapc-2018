#!/usr/bin/env python2
import commands
import random
import time

import rospy
from mac_ros_bridge.msg import SimEnd, SimStart, RequestAction
from mapc_rhbp_ettlinger.msg import AgentConfig

from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from provider.facility_provider import FacilityProvider
from provider.stats_provider import StatsProvider
from guppy import hpy

from provider.well_provider import WellProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.agent.debug')


class EvolutionaryAlgorithmManagerAgent(object):

    def __init__(self):
        rospy.init_node('debug_node', anonymous=True, log_level=rospy.ERROR)

        agent_name = rospy.get_param('~agent_name', "agentA1")

        # Handle end callback before all others (providers, ...)
        self._agent_topic_prefix = AgentUtils.get_bridge_topic_prefix(agent_name=agent_name)
        rospy.Subscriber(self._agent_topic_prefix + "end", SimEnd, self._sim_end_callback)

        self._facility_provider = FacilityProvider(agent_name=agent_name)
        self._well_provider = WellProvider(agent_name=agent_name)
        self._stats_provider = StatsProvider(agent_name=agent_name)

        self.pub_config = rospy.Publisher("/agentConfig", AgentConfig, queue_size=10)

        self.configs = []
        self.sim_start = None

        self.all_config = {
            "ShouldBidForAssemblyDecision.WEIGHT_LOAD": (-30, 30, 300),
            "ShouldBidForAssemblyDecision.WEIGHT_INGREDIENT_LOAD": (-30, 100, 1000),
            "ShouldBidForAssemblyDecision.WEIGHT_STEPS": (-100, -3, 5),
            "ShouldBidForAssemblyDecision.ACTIVATION_THRESHOLD": (-100, -15, 0),

            "ChooseStorageForHoardingDecision.WEIGHT_STEPS": (-10, -1, 10),
            "ChooseStorageForHoardingDecision.WEIGHT_ITEMS_ALREADY_THERE": (-10, -1, 10),

            "ChooseItemToGatherMechanism.WEIGHT_STEPS": (-10, 1, 10),
            "ChooseItemToGatherMechanism.WEIGHT_ASSEMBLY_ROLE_MATCH": (-10, 2.5, 25),
            "ChooseItemToGatherMechanism.WEIGHT_ASSEMBLY_ROLE_MATCH_COUNT": (-10, 0.5, 50),
            "ChooseItemToGatherMechanism.WEIGHT_PRIORITY": (-10, 200, 1000),
            "ChooseItemToGatherMechanism.FINISHED_PRODUCT_PRIORITY_TO_INGREDIENT_CONVERSION": (0, 1.9, 10),

            "ChooseBestAvailableJobDecision.DEFAULT_FINISHED_PRODUCT_GOAL": (-10, 1, 50),
            "ChooseBestAvailableJobDecision.BID_PERCENTILE": (30, 50, 99),
            "ChooseBestAvailableJobDecision.ACTIVATION_THRESHOLD": (-1000, -50, 1000),
            "ChooseBestAvailableJobDecision.IMPORTANT_JOB_THRESHOLD": (-200, 0, 1000),
            "ChooseBestAvailableJobDecision.ACTIVATION_TO_DESIRED_PRODUCT_CONVERSION": (0.0, 0.2, 4),
            "ChooseBestAvailableJobDecision.WEIGHT_PERCENTILE": (1, 10, 90),
            "ChooseBestAvailableJobDecision.WEIGHT_TIME_PASSED": (-5, -0.3, 10),
            "ChooseBestAvailableJobDecision.TIME_LEFT_WEIGHT_START": (-10, 30, 100),
            "ChooseBestAvailableJobDecision.WEIGHT_TIME_OVER": (-5, -0.4, 10),

            "ChooseBestJobAgentCombinationDecision.MIN_STEP_BUFFER": (-5, 5, 100),
            "ChooseBestJobAgentCombinationDecision.WEIGHT_LOAD": (-1, 8, 100),
            "ChooseBestJobAgentCombinationDecision.WEIGHT_INGREDIENT_LOAD": (0, 8, 100),
            "ChooseBestJobAgentCombinationDecision.WEIGHT_NO_STORAGE_NEEDED": (0, 20, 100),
            "ChooseBestJobAgentCombinationDecision.WEIGHT_STEPS": (-10, 1, 20),
            "ChooseBestJobAgentCombinationDecision.ACTIVATION_THRESHOLD": (-5000, -1000, 5000),

            "ContractNetManager.DEADLINE_BIDS": (0.5, 2, 4),
            "ContractNetManager.DEADLINE_ACKNOWLEDGEMENT": (0.5, 2, 4),

            "MainAssembleAgentDecision.WEIGHT_FINISHED_PRODUCT_FACTOR": (-5, -1, 10),
            "MainAssembleAgentDecision.WEIGHT_BID_SKILL": (-1000, -100, 1000),
            "MainAssembleAgentDecision.WEIGHT_BID_SPEED": (-1000, 100, 1000),

            "BestAgentAssemblyCombinationDecision.PRIORITY_EXPONENT": (0.2, 1, 1.5),
            "BestAgentAssemblyCombinationDecision.WEIGHT_AFTER_ASSEMBLY_ITEM_COUNT": (-10, -2.1, 10),
            "BestAgentAssemblyCombinationDecision.WEIGHT_PRIORITY": (-1, 10, 100),
            "BestAgentAssemblyCombinationDecision.WEIGHT_NUMBER_OF_AGENTS": (-100, -10, 100),
            "BestAgentAssemblyCombinationDecision.WEIGHT_PRIORITISATION_ACTIVATION": (-10, -30, 100),
            "BestAgentAssemblyCombinationDecision.WEIGHT_IDLE_STEPS": (-10, -3, 5),
            "BestAgentAssemblyCombinationDecision.WEIGHT_MAX_STEP_COUNT": (-100, -30, 10),
            "BestAgentAssemblyCombinationDecision.MAX_AGENTS": (1, 5, 17),
            "BestAgentAssemblyCombinationDecision.MIN_AGENTS": (1, 1, 4),
            "BestAgentAssemblyCombinationDecision.MAX_STEPS": (4, 10, 30),
            "BestAgentAssemblyCombinationDecision.ACTIVATION_THRESHOLD": (-2000, 300, -2000),
            "BestAgentAssemblyCombinationDecision.PREFERRED_AGENT_COUNT": (1, 4, 10),
            "BestAgentAssemblyCombinationDecision.DECISION_TIMEOUT": (1, 10, 100),
            "BestAgentAssemblyCombinationDecision.MAX_NR_OF_AGENTS_TO_CONSIDER": (7, 17, 34),
        }

        self.conf_to_adjust = {
            "ChooseBestAvailableJobDecision.PERCNTILE_TO_TRY_JOB": (0.45, 0.70, 0.99, "double"),
            "ChooseBestAvailableJobDecision.BID_PERCENTILE": (50, 85, 99, "int"),
            "ChooseBestAvailableJobDecision.TIME_LEFT_WEIGHT_START": (25, 30, 50, "int"),
            "ChooseBestAvailableJobDecision.ACTIVATION_THRESHOLD": (-200, -50, 10, "double"),
            "ChooseBestAvailableJobDecision.IMPORTANT_JOB_PERCENTILE": (0.7, 0.95, 0.999, "double"),
            "ChooseBestAvailableJobDecision.WEIGHT_PERCENTILE": (0.0, 10.0, 20.0, "double"),
            "ChooseBestAvailableJobDecision.WEIGHT_TIME_PASSED": (-4.0, -0.3, -0.01, "double"),
            "ChooseBestAvailableJobDecision.WEIGHT_TIME_OVER": (-4.0, -0.4, -0.01, "double"),
            "ChooseBestAvailableJobDecision.PRIORITISE_MISSION_JOBS": (False, True, False, "bool"),
            "ChooseBestAvailableJobDecision.WEIGHT_MISSION_JOB_PRIORITY": (0.7, 2.0, 4.0, "double"),
            "BestAgentAssemblyCombinationDecision.WEIGHT_NUMBER_OF_AGENTS": (-0.9, -0.2, -0.00, "double"),
            "BestAgentAssemblyCombinationDecision.MAX_NR_OF_AGENTS_TO_CONSIDER": (7, 17, 34, "int"),
            "BestAgentAssemblyCombinationDecision.MAX_PRIORITY_NOT_NEEDED_ITEMS": (0.05, 0.30, 0.90, "double"),
            "BestAgentAssemblyCombinationDecision.MAX_COUNT_NOT_NEEDED_ITEMS": (3, 7, 15, "int"),
            "BestAgentAssemblyCombinationDecision.MAX_AGENTS": (4, 7, 8, "int"),
            "BestAgentAssemblyCombinationDecision.MIN_AGENTS": (2, 3, 4, "int"),
            "BestAgentAssemblyCombinationDecision.MAX_STEPS": (8, 20, 30, "int"),
            "ChooseBestJobAgentCombinationDecision.PRIORITY_ACTIVATION_THRESHOLD": (-20000, -9000, -1000, "int"),
            "ChooseBestJobAgentCombinationDecision.ACTIVATION_THRESHOLD": (-100000, -30000, -5000, "int")
        }

        rospy.Subscriber(self._agent_topic_prefix + "request_action", RequestAction, self._request_action_callback)
        rospy.Subscriber(self._agent_topic_prefix + "start", SimStart, self._sim_start_callback)

    def _sim_end_callback(self, sim_end):
        """

        :param sim_end:
        :type sim_end: SimEnd
        :return:
        """
        rospy.logerr("EvolutionaryAlgorithmManagerAgent:: Ending simulation: %s (%s)", self.sim_start.map, self.sim_start.simulation_id)
        current_config = self.current_config
        current_config["___massium"] = self._stats_provider.massium
        current_config["___score"] = self._stats_provider.score
        current_fitness =  self._stats_provider.massium
        for well in self._facility_provider.own_wells.values():
            possible_well = self._well_provider.get_well(well.type)
            current_fitness += possible_well.cost
        current_config["___fitness"] = current_fitness
        current_config["___timestamp"] = str(time.time())
        self.configs.append(current_config)

        # Sort by fitness
        self.configs.sort(key=lambda k: k['___fitness'], reverse=True)

        # Only keep 10 best configs
        self.configs = self.configs[:10]

        config = random.choice(self.configs)

        config = self.mutate(config)

        for key, value in config.iteritems():
            rospy.set_param(key, value)

        current_config["sim_start"] = self.sim_start

        for key, value in current_config.iteritems():
            rospy.logerr("EvolutionaryAlgorithmManagerAgent:: -- Config param %s: %s", key, str(value))

    def _request_action_callback(self, requestAction):
        """

        :param requestAction: RequestAction
        :return:
        """
        # free_mem = commands.getstatusoutput("free | grep Mem | awk '{print $4/$2 * 100.0}'")
        # free_swap =  commands.getstatusoutput("free | grep Swap | awk '{print $4/$2 * 100.0}'")
        # agent_mem_usage =  commands.getstatusoutput("ps aux | grep mac_ros_bridge | awk '{print $4}'")
        # bridge_mem_usage =  commands.getstatusoutput("ps aux | grep rhbp_agent.py | awk '{print $4}'")
        # ettilog.logerr("EvolutionaryAlgorithmManagerAgent:: Memory free %s", free_mem)
        # ettilog.logerr("EvolutionaryAlgorithmManagerAgent:: Swap free %s", free_swap)
        # ettilog.logerr("EvolutionaryAlgorithmManagerAgent:: agent_mem_usage %s", agent_mem_usage)
        # ettilog.logerr("EvolutionaryAlgorithmManagerAgent:: bridge_mem_usage %s", bridge_mem_usage)

        pass

    def _sim_start_callback(self, sim_start):
        """

        :param sim_start:
        :type sim_start: SimStart
        :return:
        """

        rospy.logerr("EvolutionaryAlgorithmManagerAgent:: Starting simulation: %s (%s)", sim_start.map, sim_start.simulation_id)
        self.sim_start = sim_start
        self.current_config = self._read_config()

    def _read_config(self):
        res = {}
        for key in self.conf_to_adjust.keys():
            res[key] = rospy.get_param(key)

        return res

    def mutate(self, config):
        new_config = {}

        for key, value in config.iteritems():
            if key not in self.conf_to_adjust:
                continue
            MUTATION_RATE = 0.05

            if self.conf_to_adjust[key][3] == "bool":
                if random.random() < MUTATION_RATE:
                    value = not value
            elif self.conf_to_adjust[key][3] in ["int", "double"]:
                range = self.conf_to_adjust[key][2] - self.conf_to_adjust[key][0]
                max_mutation = MUTATION_RATE * range

                value += random.uniform(-max_mutation, max_mutation)
                value = max(self.conf_to_adjust[key][0], value)
                value = min(self.conf_to_adjust[key][2], value)

                if self.conf_to_adjust[key][3] == "int":
                    int_value = int(value)

                    prob_to_add = (value - int_value)
                    int_value += random.random() < prob_to_add

                    value = int_value

                new_config[key] = value

        return new_config


if __name__ == '__main__':

    try:
        controler = EvolutionaryAlgorithmManagerAgent()
        rospy.spin()

    except rospy.ROSInterruptException:
        ettilog.logerr("program interrupted before completion")
