from __future__ import division


class AgentUtils(object):

    @staticmethod
    def get_bridge_topic_prefix(agent_name):
        """
        Determine the topic prefix for all topics of the bridge node corresponding to the agent
        :param agent_name: current agents name
        :return: prefix just before the topic name of the bridge
        """
        return '/bridge_node_' + agent_name + '/'

    @staticmethod
    def get_bridge_topic_agent(agent_name):
        """
        Determine the topic prefix for all topics of the bridge node corresponding to the agent
        :param agent_name: current agents name
        :return: prefix just before the topic name of the bridge
        """
        return AgentUtils.get_bridge_topic(agent_name, "agent")

    @classmethod
    def get_bridge_topic(cls, agent_name, postfix):
        """
        Returns the bridge topic of an agent together with a postfix
        :param agent_name: current agents name
        :param postfix: postfix
        :return: str
        """
        return AgentUtils.get_bridge_topic_prefix(agent_name=agent_name) + postfix

    @classmethod
    def get_coordination_topic(cls, task_type, message_type):
        """
        Topic used for coordination in contract net
        :return:
        """
        return '/coordination/%s/%s'%(task_type, message_type)