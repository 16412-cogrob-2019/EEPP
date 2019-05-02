# path_planning

## Overview

This is the Energy Efficient Path Planning (EEPP) teams contribution to the 2019 Cognitive Robotics Grand Challenge.

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [ROS map_server](http://wiki.ros.org/map_server)

		sudo apt-get install ros-kinetic-map-server


## Usage
Run the main node with

	roslaunch path_planning EEPP.launch

If you also want to launch the map server, use

	roslaunch path_planning EEPP.launch map_server:=true

If you also want to launch the testing node, use

	roslaunch path_planning EEPP.launch test:=true


## Nodes
---
### EEPP_node

Reads point of interest data and plans paths for each robot to each point of interest. Publishes the point of interest data with added paths and path costs


#### Subscribed Topics

* **`/maas/poi_data`** ([std_msgs/String])

	The json dump containing poi data


#### Published Topics

* **`/eepp/path_data`** ([std_msgs/String])

	The json dump containing poi data with added paths and path costs

---

### testing_node

This node can be used for testing the EEPP_node. It simulates the MAAS team publishing poi data, by generating random PoI's at a given rate. It also simulates the MCTS team subscribing to the path_data topic.


#### Subscribed Topics

* **`/eepp/path_data`** ([std_msgs/String])

	The json dump containing poi data with added paths and path costs


#### Published Topics

* **`/maas/poi_data`** ([std_msgs/String])

	The json dump containing poi data
