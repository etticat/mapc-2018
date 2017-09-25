# Multi-Agent Programming Contest Workspace

This is the main workspace for the TUB participation in the Multi-Agent Contest 2017.

It uses the ROS Hybrid Behaviour Planner Framework (RHBP) on top of ROS (Robot Operating System).

## Setup and Install

### Dependencies

```
# graphhopper requires
sudo apt install maven default-jdk

# tub_contest requires
sudo apt install python-pip python-pandas
# rhbp requires ROS
# Follow instructions for ROS Kinetic here: http://wiki.ros.org/kinetic/Installation/Ubuntu
sudo apt install ros-kinetic-desktop
# and
sudo apt install python-pip
pip install lindypy

```

### Clone and build

1. `git clone --recursive git@gitlab.tubit.tu-berlin.de:mac17/mac_workspace.git`
2. `cd mac_workspace`
3. `catkin_make`
4. Download the latest massim release https://github.com/agentcontest/massim/releases
5. Adjust the script "script/script/start_massim.sh" to use the correct release.
6. If you want to use the RHBP rqt plugin you have to start rqt once with `rqt --force-discover`

## Execution and further Documentation

Please refer to the [tub_contest package](src/tub_contest/Readme.md) for further instruction on how to launch our solution.


## Packages and Workspace Structure

* `script` useful scipts, you might want to add this directory to your PATH variable (already done in VM).
* `src/rhbp` git submodule of the used RHBP framework.
* `src/mac_ros_bridge` ROS package that includes a proxy ROS node that works as a bridge between the massim simulation server and ROS. It converts all simulation perception and creates all required topics from the configuration.
* `src/mac_rhbp_example` example agent implementation using RHBP and ROS.
