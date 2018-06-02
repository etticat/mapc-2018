# mapc_rhbp_ettlinger

This is a very basic example that shows how RHBP can be used within the Multiagent Programming Contest Scenario of 2018.

The implemented agents are traveling to randomly selected shops. 
If the battery level is getting to low the agents will also recharge at a randomly selected charging station.

## Behaviour Model

![alt text](doc/mapc_rhbp_ettlinger.png)

## Execution

The example can be executed with `roslaunch mapc_rhbp_ettlinger rhbp_agents_example.launch`
The configuration in above launch file is made for a 6 agent scenario of default team A executed on localhost.

## Exercise

A simple exercise using this example can be to implement a more useful selection of charging and shop destinations.


