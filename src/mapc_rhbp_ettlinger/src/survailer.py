#!/usr/bin/env python2
import random
import time

import rospy
from mac_ros_bridge.msg import SimEnd, SimStart
from mapc_rhbp_ettlinger.msg import AgentConfig

from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from provider.stats_provider import StatsProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.agent.debug')

class ControlAgent(object):

    def __init__(self):
        rospy.init_node('debug_node', anonymous=True, log_level=rospy.ERROR)

        agent_name = rospy.get_param('~agent_name', "agentA1")
        self.stas_provider = StatsProvider(agent_name=agent_name)
        self._agent_topic_prefix = AgentUtils.get_bridge_topic_prefix(agent_name=agent_name)

        rospy.Subscriber(self._agent_topic_prefix + "start", SimStart, self._sim_start_callback)
        rospy.Subscriber(self._agent_topic_prefix + "end", SimEnd, self._sim_end_callback)
        self.pub_config = rospy.Publisher("/agentConfig", AgentConfig, queue_size=10)
        
        self.configs = []
        self.sim_start = None

        
        self.all_config = {
            "ShouldBidForAssemblyDecision.WEIGHT_LOAD": (-30,30,300),
            "ShouldBidForAssemblyDecision.WEIGHT_INGREDIENT_LOAD": (-30,100,1000),
            "ShouldBidForAssemblyDecision.WEIGHT_STEPS": (-100,-3, 5),
            "ShouldBidForAssemblyDecision.ACTIVATION_THRESHOLD": (-1000, 0, 1000),

            "ChooseStorageForHoardingDecision.WEIGHT_STEPS": (-10, -1, 10),
            "ChooseStorageForHoardingDecision.WEIGHT_ITEMS_ALREADY_THERE": (-10, -1, 10),

            "ChooseItemToGatherMechanism.WEIGHT_STEPS": (-10, 1, 10),
            "ChooseItemToGatherMechanism.WEIGHT_ASSEMBLY_ROLE_MATCH": (-10, 2.5, 25),
            "ChooseItemToGatherMechanism.WEIGHT_ASSEMBLY_ROLE_MATCH_COUNT": (-10, 0.5, 50),
            "ChooseItemToGatherMechanism.WEIGHT_PRIORITY": (-10, 200, 1000),
            "ChooseItemToGatherMechanism.FINISHED_PRODUCT_PRIORITY_TO_INGREDIENT_CONVERSION": (0, 1.9, 10),

            "ChooseBestAvailableJobDecision.DEFAULT_FINISHED_PRODUCT_GOAL": (-10, 1, 50),
            "ChooseBestAvailableJobDecision.BID_PERCENTILE": (30, 50, 90),
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

            "BestAgentAssemblyCombinationDecision.PRIORITY_EXPONENT" : (0.2, 1, 1.5),
            "BestAgentAssemblyCombinationDecision.WEIGHT_AFTER_ASSEMBLY_ITEM_COUNT" : (-10, -2.1, 10),
            "BestAgentAssemblyCombinationDecision.WEIGHT_PRIORITY" : (-1, 10, 100),
            "BestAgentAssemblyCombinationDecision.WEIGHT_NUMBER_OF_AGENTS" : (-100, -10, 100),
            "BestAgentAssemblyCombinationDecision.WEIGHT_PRIORITISATION_ACTIVATION" : (-10, -30, 100),
            "BestAgentAssemblyCombinationDecision.WEIGHT_IDLE_STEPS" : (-10, -3, 5),
            "BestAgentAssemblyCombinationDecision.WEIGHT_MAX_STEP_COUNT" : (-100, -30, 10),
            "BestAgentAssemblyCombinationDecision.MAX_AGENTS" : (1,5,17),
            "BestAgentAssemblyCombinationDecision.MIN_AGENTS" : (1,1,4),
            "BestAgentAssemblyCombinationDecision.MAX_STEPS" :  (4,10,30),
            "BestAgentAssemblyCombinationDecision.ACTIVATION_THRESHOLD" :  (-2000, 300, -2000),
            "BestAgentAssemblyCombinationDecision.PREFERRED_AGENT_COUNT" : (1,4,10),
        }

        self.conf_to_adjust = {
            "BestAgentAssemblyCombinationDecision.PRIORITY_EXPONENT": (0.2, 1, 2),
            "BestAgentAssemblyCombinationDecision.WEIGHT_PRIORITY": (-1, 100, 1000),
            "BestAgentAssemblyCombinationDecision.WEIGHT_NUMBER_OF_AGENTS": (-50, -4, 5),
            "BestAgentAssemblyCombinationDecision.WEIGHT_PRIORITISATION_ACTIVATION": (-10, 30, 100),
            "BestAgentAssemblyCombinationDecision.WEIGHT_IDLE_STEPS": (-10, -3, 5),
            "BestAgentAssemblyCombinationDecision.WEIGHT_MAX_STEP_COUNT": (-100, -2, 10),
            "BestAgentAssemblyCombinationDecision.MAX_AGENTS": (1, 7, 17),
            "BestAgentAssemblyCombinationDecision.MIN_AGENTS": (1, 1, 4),
            "BestAgentAssemblyCombinationDecision.MAX_STEPS": (4, 10, 30),
            "ChooseBestAvailableJobDecision.IMPORTANT_JOB_THRESHOLD": (-200, 0, 1000),
        }

    def _sim_end_callback(self, sim_end):
        """

        :param sim_end:
        :type sim_end: SimEnd
        :return:
        """
        end_massium = self.stas_provider.massium
        current_config = self.current_config
        current_fitness = end_massium
        current_config["___fitness"] = current_fitness
        current_config["___timestamp"] = str(time.time())
        self.configs.append((current_fitness, current_config))

        # Sort by fitness
        self.configs.sort(key=lambda k: k['fitness'], reverse=True)

        # Only keep 10 best configs
        self.configs = self.configs[:10]

        fitness, config = random.choice(self.configs)

        config = self.mutate(config)

        for key, value in config.iteritems():
            rospy.set_param(key, value)

        fh = open("results.txt", "a")
        for key, value in current_config.iteritems():
            fh.write(key +":" + str(value) + ",")
        fh.write("sim_start:" + str(self.sim_start))
        fh.write("\n")
        fh.close()

    def _sim_start_callback(self, sim_start):
        """

        :param sim_start:
        :type sim_start: SimStart
        :return:
        """

        rospy.logerr("Starting simulation: %s (%s)", sim_start.map, sim_start.simulation_id)
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
            range = self.conf_to_adjust[key][2] - self.conf_to_adjust[key][0]
            max_mutation = 0.05 * range

            value += random.uniform(-max_mutation, max_mutation)
            value = max(self.conf_to_adjust[key][0], value)
            value = min(self.conf_to_adjust[key][2], value)

            new_config[key] = value

        return new_config


if __name__ == '__main__':

    try:
        controler = ControlAgent()
        rospy.spin()

    except rospy.ROSInterruptException:
        ettilog.logerr("program interrupted before completion")

