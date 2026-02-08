# 🤖 final_project
This repository contains all the packages I have created or modified to run my final robotics lab project. Make sure to also have the standard aruco ros package to detect aruco markers.

This project simulates the enviroment of an automatized cafeteria.

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
