# ROS Nodes for NaturalPoint Optitrack Motion Capture System

A ROS package for extracting streaming information from Optitrack system and publishing rigid-body and free marker information.

This has been tested in the Stanford ASL setup with Motive:Tracker 2.2.0 and ROS Noetic.


---
## Installation and Setup
Clone this repository to your ROS `[ROS_WS]/src` folder and then run `catkin_make` to build the package.

### ROS Configuration
Modify the `mocap_optitrack/config/mocap.yaml` configuration file. The `optitrack_config` parameters in this file are currently set for the Stanford ASL setup. Set `free_markers` to `true` if you want all unlabeled markers (not belonging to rigid bodies) to be published. Define a new rigid-body for each rigid-body that is being tracked by the Optitrack system that you want published. The number corresponds to the `Streaming ID` value set in the Motive:Tracker software for the particular asset. The remaining parameters for each rigid-body specify the ROS topic name information.

### Motive:Tracker Configuration
Create any assets in the scene that you want to stream information for.

Enable `Broadcast Frame Data`

Under `View -> Data Streaming Pane` make sure `Local Interface` is set to the IP address of the computer running Motive:Tracker corresponding to the network that is used to share the data stream.

Enable `Rigid Bodies` and `Unlabeled Markers`

Set `Transmission Type` to `Multicast`

Ensure the `Command Port`, `Data Port`, and `Multicast Interface` match the values in the `mocap_optitrack/config/mocap.yaml` configuration file.

---
### Use
Run the mocap ROS node by:
```
roslaunch mocap_optitrack mocap.launch
```
