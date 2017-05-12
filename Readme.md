# Multi-Agent Programming Contest Workspace

This is a workspace for the TUB participation in the Multi-Agent Contest 2017.

It uses the ROS Hybrid Behaviour Planner Framework (RHBP) on top of ROS (Robot Operating System).

## Setup and Install

1. `git clone --recursive git@gitlab.tubit.tu-berlin.de:mac17/mac_workspace.git`
2. `cd mac_workspace`
3. `catkin_make`
4. Download the latest massim release https://github.com/agentcontest/massim/releases
5. Adjust the script "script/script/start_massim.sh" to use the correct release.
6. If you want to use the RHBP rqt plugin you have to start rqt once with `rqt --force-discover`
7. Clone your own group repository into the src directory
```
cd src
git clone git@gitlab.tubit.tu-berlin.de:mac17/group1.git
# or
git clone git@gitlab.tubit.tu-berlin.de:mac17/group2.git
```
8. `catkin_make` to build your group package as well

## Execution and Usage

1. First terminal tab: `pycharm` or `charm`
2. Second terminal tab: `roscore`
3. Third terminal tab: `rqt`
4. Fourth terminal tab: `script/script/start_massim.sh`
    1. Select configuration, e.g. "[ NORMAL  ]  ##   0 conf/SampleConfig.json"
    2. Press ENTER to start simulation
5. Start actual agent implementation (select one of the following options)
    1. Dummy implementation only doing skip: `roslaunch mac_ros_bridge mac_ros_bridge_example.launch`
    2. RHBP example implementation `roslaunch mac_rhbp_example rhbp_agents_example.launch
`

## Packages and Workspace Structure

* `script` useful scipts, you might want to add this directory to your PATH variable (already done in VM).
* `src/rhbp` git submodule of the used RHBP framework.
* `src/mac_ros_bridge` ROS package that includes a proxy ROS node that works as a bridge between the massim simulation server and ROS. It converts all simulation perception and creates all required topics from the configuration.
* `src/mac_rhbp_example` example agent implementation using RHBP and ROS.
