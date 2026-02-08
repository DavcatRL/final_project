# 🤖 Final RL project: introduction

The project simulates the enviroment of an automatized cafeteria. The two robots chosen to represent the barman and the waiter are Iiwa and fra2mo respectively. They act together to take the user's order and deliver it to the correct table.

# Description of the packages 

In this repository you will find 5 different packages:
- ros2_fra2mo:             the package that describes fra2mo;
- ros2_iiwa:               the package that describes Iiwa;
- ros2_kdl_package:        contains the bar_client node and the source file that defines the controller of the Iiwa robot;
- ros2_package_interfaces: contains the definition of the MarkerActivateCommand msg;
- final_project:           contains everything else that is needed for the project.

Make sure to also have installed the standard aruco_ros package for the detection of aruco markers.

# Run the simulation

To run the simulation, you only have to execute the following commands in two different terminals:

```
ros2 launch final_project bar.launch.py
```

This will spawn in Gazebo iiwa and fra2mo in their correct configuration and all the nodes required to complete the task. The second one:

```
ros2 run ros2_kdl_package bar_client --ros-args -p use_sim_time:=true
```

will run the node that you'll need to interrface with the simulation you've just launched.

Then, you only have to follow the instructions that will appear on the terminal where you runned the second command and the simulation will start.
