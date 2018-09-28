# Team TUBDAI submission for Multi Agent Programming Contest 2018

This is the submission for the Multi-Agent Programming Contest 2018 for the team TUBDAI. It applies the ROS Hybrid Behaviour Planner Framework (RHBP) on top of the ROS (Robot Operating System) framework.

Technische Universität Berlin - DAI-Labor - http://www.dai-labor.de/

Contest homepage: https://multiagentcontest.org/2018/

## Setup and Install

### Dependencies

Graphhopper requires
```
sudo apt install maven default-jdk
```
mapc_utils requires

```
sudo apt install python-pip python-pandas python-urllib3
```
RHBP requires ROS. Follow instructions for ROS Kinetic here: http://wiki.ros.org/kinetic/Installation/Ubuntu
```
sudo apt install ros-kinetic-desktop python-pip pip install lindypy
```

### Directory Structure

You can get the MAPC simulation server from `https://github.com/agentcontest/massim/releases` as a release version.

 * mapc_workspace
     * src
        * rhbp
        * rhbp_self_organisation
        * mapc_rhbp_ettlinger: Main project folder
            * src: Source code
            * launch: Launch files
            * ...
        * mac_ros_bridge: ROS package that includes a proxy ROS node that works as a bridge between the massim simulation server and ROS. It converts all simulation perception and creates all required topics from the configuration.
     * script: Helper scripts to start massim
     * thirt-party: Thirt party tools
 * massim
     * massim-2018-\*.\*
        * server
        * ...

### Clone and build

1. `git clone --recursive git@github.com:etticat/mapc-2018.git`
2. `cd mac_workspace`
3. `catkin_make`
4. Download the latest MASSIM release https://github.com/agentcontest/massim/releases
5. Adjust the script "script/script/start_massim.sh" to use the correct release if necessary.
7. If you want to use the RHBP rqt plugin you have to start rqt once with `rqt --force-discover`


## Execution and further Documentation

### massim Server

You can start the massim server with one of the following commands, depending on your workspace.

Using binary massim distribution with
`script/start_massim.sh`
Or running directly from sources with `script/start_massim_src.sh`
The second option requires that you have once executed `mvn install` in the massim sources root.


### Agents

* Make sure the  launch file  `src/mapc_rhbp_ettlinger/src/launch/start_big_prod_contest.launch` contains the correct connection strings and credentials.
* start the agents using `roslaunch mapc_rhbp_ettlinger start_big_prod.launch`
