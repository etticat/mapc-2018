# mac_ros_bridge

 ROS package that includes a proxy ROS node that works as a bridge between the massim simulation server and ROS. 
 It converts all simulation perception and creates all required topics from the configuration.
 
# Topics

The mac_ros_bridge provides a communication bridge per agent in the MAPC scenario.
It provides topics either on the individual agent level or global level
* Individual level: `/bridge_node_agentA1/agent`, with `agentA1` as the configured AGENT_NAME
* Global level: `/team` with all agents publishing on the same topic. Information filtering would be required to 
  do in the particular client.

The most important topics are `/bridge_node_AGENT_NAME/request_action` that is triggered by the massim `REQUEST-ACTION` 
on every simulation cycle per agent and `/bridge_node_AGENT_NAME/generic_action` that is subscribed by the 
mac_ros_bridge to receive the action to be executed by the agent. The `request_action` message does also already include most of the
perception.

The following topics are available.

##Simulation:

* `/bridge_node_AGENT_NAME/request_action` : Called before every simulation cycle. Trigger for the decision-making. Also includes perception.
* `/bridge_node_AGENT_NAME/start`: Called when a simulation round is started, here you probably want to setup your agent.
* `/bridge_node_AGENT_NAME/bye`: message when all matches have been run, just before the server closes all sockets
* `/bridge_node_AGENT_NAME/end`: Message after each simulation


##Perception

`resource`, `well` , and `entity` are available on global as well as local level. Local agent level includes the individual
agent perception while all agents publish additionally on the global level like `/resource` to share the information.

* `/bridge_node_AGENT_NAME/resource`: Local resource perception of the agent
* `/bridge_node_AGENT_NAME/well`: Local well perception of the agent
* `/bridge_node_AGENT_NAME/entity`: Local entity perception of the agent

* `/bridge_node_AGENT_NAME/agent`: Message for an individual agent
* `/charging_station`: Messages for all charging stations
* `/dump`: Messages for all dump locations
* `/entity`: Messages for all agent entities
* `/mission_job`: Messages for available mission jobs
* `/priced_job`: Messages for all available priced jobs
* `/auction_job`: Messages for all available auction jobs
* `/resource`: Messages for all resources
* `/shop`: Messages for all shops
* `/storage`: Messages for all storages
* `/team`: Message about team
* `/workshop`: Messages for all workshops
