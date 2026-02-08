# ros2_package_interfaces

## :package: About

This package contains the *custom interface* tutorial code to create and run your first publisher and subscriber node using C++ with custom interfaces.

Created following the official [ROS 2 Documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)

## :hammer: Build
Clone this package in the `src` folder of your ROS 2 workspace. Check for missing dependencies
```
$ rosdep install -i --from-path src --rosdistro humble -y
```
Build your new package
```
$ colcon build --packages-select ros2_package_interfaces
```

## :white_check_mark: Usage
Source the setup files
```
$ . install/setup.bash
```
To use it in your C++ package, add these lines to the *CMakeLists.txt* of your package
```
find_package(ros2_package_interfaces REQUIRED) 
...
ament_target_dependencies(your_package_node ros2_package_interfaces)
```
Finally, specify the dependency of your package in the *package.xml* file
```
<depend>ros2_package_interfaces</depend>
```
You can find an example [here](https://github.com/RoboticsLab2025/ros2_package).