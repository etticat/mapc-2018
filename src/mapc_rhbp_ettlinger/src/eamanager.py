#!/usr/bin/env python2
import random
import time

import rospy
from mac_ros_bridge.msg import SimEnd, SimStart, RequestAction
from mapc_rhbp_ettlinger.msg import AgentConfig

from common_utils import etti_logging
from common_utils.agent_utils import AgentUtils
from provider.facility_provider import FacilityProvider
from provider.stats_provider import StatsProvider
from provider.well_provider import WellProvider

ettilog = etti_logging.LogManager(logger_name=etti_logging.LOGGER_DEFAULT_NAME + '.agent.debug')


class EvolutionaryAlgorithmManagerAgent(object):
    """
    Ros node, that uses an evolutionary algorithm in an effort to try to improve the performance by tweaking certain
    config parameters.
    """

    MUTATION_RATE = 0.05

    def __init__(self):
        rospy.init_node('ea_node', anonymous=True, log_level=rospy.ERROR)

        agent_name = rospy.get_param('~agent_name', "agentA1")

        # Handle end callback before all others (providers, ...)
        rospy.Subscriber(AgentUtils.get_bridge_topic(agent_name=agent_name, postfix="end"), SimEnd,
                         self._sim_end_callback)

        # Init providers
        self._facility_provider = FacilityProvider(agent_name=agent_name)
        self._well_provider = WellProvider(agent_name=agent_name)
        self._stats_provider = StatsProvider(agent_name=agent_name)

        # Array of all configs, the alogirhm has tried with its corresponding fitness values
        self.configs = []
        self.sim_start = None

        # The config parameters, which should be adjusted
        # The value is a tuple with
        # - [0] -> lower bound
        # - [1] -> start value
        # - [2] -> upper bound
        # - [3] -> data type
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

        rospy.Subscriber(AgentUtils.get_bridge_topic(agent_name=agent_name, postfix="start"), SimStart,
                         self._sim_start_callback)

    def _sim_end_callback(self, sim_end):
        """
        At the end of a simulation, save all config parameters and outcomes (for detailed investigation).
        Also mutate one of the best configs, to find an even better one.
        :param sim_end:
        :type sim_end: SimEnd
        :return:
        """
        rospy.logerr("EvolutionaryAlgorithmManagerAgent:: Ending simulation: %s (%s)", self.sim_start.map,
                     self.sim_start.simulation_id)

        last_config = self.current_config
        last_config["___massium"] = self._stats_provider.massium
        last_config["___score"] = self._stats_provider.score
        last_config["sim_start"] = self.sim_start
        current_fitness = self._stats_provider.massium

        # Add the cost of all built wells to the total fitness value
        for well in self._facility_provider.own_wells.values():
            possible_well = self._well_provider.get_well(well.type)
            current_fitness += possible_well.cost

        last_config["___fitness"] = current_fitness
        last_config["___timestamp"] = str(time.time())

        # Add the current config to all configs
        self.configs.append(last_config)

        # Sort by fitness
        self.configs.sort(key=lambda k: k['___fitness'], reverse=True)

        # Pick a random config from the 10 best configs
        new_config = random.choice(self.configs[:10])

        # Mutate the config
        new_config = self.mutate(new_config)

        # Apply the mutated config for the next round
        for key, value in new_config.iteritems():
            rospy.set_param(key, value)

        # Print all values of the last config
        for key, value in last_config.iteritems():
            rospy.logerr("EvolutionaryAlgorithmManagerAgent:: -- Config param %s: %s", key, str(value))

    def _sim_start_callback(self, sim_start):
        """
        Read the current config values when the simulation starts
        :param sim_start:
        :type sim_start: SimStart
        :return:
        """

        rospy.logerr("EvolutionaryAlgorithmManagerAgent:: Starting simulation: %s (%s)", sim_start.map,
                     sim_start.simulation_id)
        self.sim_start = sim_start
        self.current_config = self._read_config()

    def _read_config(self):
        """
        Reads the config parameters from rospy
        :return:
        """
        res = {}
        for key in self.conf_to_adjust.keys():
            res[key] = rospy.get_param(key)

        return res

    def mutate(self, config):
        """
        Slightly mutates a config to create a new (and hopefully better) config
        :param config:
        :return:
        """
        new_config = {}

        for key, value in config.iteritems():
            # Ignore all configs/other values that are not part of the parameters we want to adjust (e.g. fitness, ..)
            if key not in self.conf_to_adjust:
                continue

            if self.conf_to_adjust[key][3] == "bool":
                # For bool values we change the value with the probability of the Mutation rate
                if random.random() < EvolutionaryAlgorithmManagerAgent.MUTATION_RATE:
                    value = not value
                    new_config[key] = value
            elif self.conf_to_adjust[key][3] in ["int", "double"]:
                # For numbers we add/subtract a random value
                range = self.conf_to_adjust[key][2] - self.conf_to_adjust[key][0]
                max_mutation = EvolutionaryAlgorithmManagerAgent.MUTATION_RATE * range

                value += random.uniform(-max_mutation, max_mutation)
                value = max(self.conf_to_adjust[key][0], value)
                value = min(self.conf_to_adjust[key][2], value)

                # make sure int values stay int values
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
