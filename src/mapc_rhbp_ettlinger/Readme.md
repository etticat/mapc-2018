# mapc_rhbp_ettlinger

This is a very basic example that shows how RHBP can be used within the Multiagent Programming Contest Scenario of 2018.

The implemented agents are traveling to randomly selected shops. 
If the battery level is getting to low the agents will also recharge at a randomly selected charging station.

## Behaviour Model

![alt text](doc/mapc_rhbp_ettlinger.png)

## Execution

* Start massim contest server `sh script/start_massim_contest.sh`
* start agents: 
  * either together with planner: 
    * `roslaunch mapc_rhbp_ettlinger start_big_prod.launch`
  * seperately from planner: 
    * `roslaunch mapc_rhbp_ettlinger start_big_dev.launch`
    * `roslaunch mapc_rhbp_ettlinger planner.launch`
* start item observer: 
  * `roslaunch mapc_rhbp_ettlinger debug_agent.launch`
* go back to second tab (agents) and wait for all agents being initialised (Indicated by "Initialisation finished" message)
* Go to first terminal and hit Enter to start contest
* Overview of items and goals can be found in 3rd window



## Exercise

A simple exercise using this example can be to implement a more useful selection of charging and shop destinations.


