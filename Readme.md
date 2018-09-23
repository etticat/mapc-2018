# Multi-Agent Programming Contest Workspace

Technische Universität Berlin - DAI-Labor - http://www.dai-labor.de/

Contest homepage: https://multiagentcontest.org/2018/

This is the main workspace for the TUB participation in the Multi-Agent Contest (MAPC) 2018.
It applies the ROS Hybrid Behaviour Planner Framework (RHBP) on top of the ROS (Robot Operating System) framework.

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

### Expected directory Structure

You can get the MAPC simulation server from `https://github.com/agentcontest/massim/releases` as a release version for

following Option 1) or using the sources (`git clone https://github.com/agentcontest/massim.git massim18`) for Option 2)

***Option 1)***

* your workspace
    * mapc_workspace
    * massim
        * massim-2018-\*.\*
           * server
           * ...

***Option 2)***

An alternative project structure working with massim sources has to look like below

* your workspace
    * mapc_workspace
    * massim
        * massim18 (src root directory )
            * server
            * ...

### Clone and build

1. `git clone --recursive git@gitlab.tubit.tu-berlin.de:asp_b_ss18/mapc_workspace.git`
2. `cd mac_workspace`
3. `catkin_make`
4. Download the latest MASSIM release https://github.com/agentcontest/massim/releases
5. Adjust the script "script/script/start_massim.sh" to use the correct release if necessary.
6. If you want to use the RHBP rqt plugin you have to start rqt once with `rqt --force-discover`

## Execution and further Documentation

### massim Server

You can start the massim server with one of the following commands, depending on your workspace.

Using binary massim distribution with
`script/start_massim.sh`
Or running directly from sources with `script/start_massim_src.sh`
The second option requires that you have once executed `mvn install` in the massim sources root.

## Packages and Workspace Structure

* `script` useful scipts, you might want to add this directory to your PATH variable.
* `third-party` third-party module dependencies that are referenced as git submodules
* `src/rhbp` git submodule of the RHBP framework.
* `src/mac_ros_bridge` ROS package that includes a proxy ROS node that works as a bridge between the massim simulation server and ROS. It converts all simulation perception and creates all required topics from the configuration.
* `src/mac_rhbp_example` example MAPC agent implementation using the mac_ros_bridge, RHBP and ROS.

\section{Files}
!!! This may not make it into the final thesis but into the readme file of the project
* data/osm -> contains map data for graphopper distance calculation
* launch -> contains launch files for different configurations
** e.g. start \_big \_prod.launch starts 34 agents including all other nodes (ea, planner, debug \_node)
** start \_small \_dev starts only 6 agents without additional nodes
** ... (if interresting)
** python \_logging.conf -> configuration for what should be logged and what not 	
** Individual node start files: rhbp \_agent.launch, eamanager.launch, debug \_agent.launch 
* msg contains all Messages that are sent within the project for coordination, ..
* src contains all souces
* srv contains all service information (Graphhopper Service)
* CMakeLists.txt -> make file for the project
* package.xml -> contains all packages, mantainer infos, ..
* setup.py -> info on which folders are compiled
