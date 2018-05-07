
# mac_utils

Collection of useful utility modules for the Multi-agent Programming Contest

## graphhopper
 
### PathPlanner
 
PathPlanner is a utility class that can be used to calculate role dependent
distances in the MAPC.

**Example:**

```
planner = PathPlanner(role=Role(name='truck', speed=100), cell_size=your_cell_size, 
                      proximity=your_proximity)
# you can also switch the map during run time
planner.set_map('london')
start = Position(lat=51.4773216248, long=-0.194169998169)
destination=Position(lat=51.4842300415,long=-0.160270005465)
distance = planner.distance(start,destination)
```
 
### GraphhopperProcessHandler
 
PathPlanner requires a running GraphhopperProcessHandler, which is managing
the launch and stop of GraphHopper map servers. The GraphHopper map servers
are used to calculate the routes in the end.

Just use the `graphhopper.launch` to start the node. The launch files does
also allow to specify the server ports as well as prestarting several map
server instances for faster access. Map files have to be placed into
data/osm/*.osm.pbf
 
## statistics

Requires `sudo apt install python-pandas`

A python module that simplifies the collection of statistics in a discrete simulation environment
such as the MAPC. 

**Example:**

```
stats = Statistics()

for i in range(10):
    # Update step has to be called once in each discrete simulation step
    stats.update_step(i)
    # Arbitrary key value pairs can be collected to generate statistics about them
    stats.add_value('a', i)
    stats.add_value('b', i / 2.0)
    # increment the value of a key from the former simulation step
    stats.increment_last_value('c', 1)
    # increment the value of a key from the current simulation step
    stats.increment_current_value('d', 1)
    stats.increment_current_value('d', 1)

# finalise the statistics collection. This has to be called before further
# statistics generation
stats.finalise()

# Accesss the pandas data frame
print(stats.get_panda_frame())

# Generate some descripte statistics
print(stats.get_descriptive_stats())

# Export a plot and a csv file
# Files will be created in ~/.ROS/NODE_NAME
stats.write_to_csv('test')
stats.write_to_plot('test')
```