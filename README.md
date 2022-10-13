# abb_cam Repository
This repository includes adapted ROS interface for 2 ABB manipulators (IRB2600/IRB120)

## Content
1. [ABB IRB2600](#abb-irb2600)
1. [ABB IRB120](#abb-irb120)

## Dependencies

### Quick Configre with ROS install
In this directory([Workspace]/src/abb_cam), run following command in a terminal:  
```
rosinstall .
```  
### List
- [ROS Industrial Core](https://github.com/ros-industrial/industrial_core)

## ABB IRB2600
[ROS packages](./irb2600/)
### Launch ROS Interfac
To enable ROS interface, run:  
```
roslaunch abb_irb2600_support robot_interface_download_irb2600_12_165.launch robot_ip:=[robot IP]
```

or include the above launch file:
```xml
<include file="$(find abb_irb2600_support)/launch/robot_interface_download_irb2600_12_165.launch" >
    <arg name="robot_ip" value="192.168.10.26"/>
</include>
```


### ROS Interface
- Current joint states: (ROS topic)  
  `/irb2600/joint_states` 
- Send spline in joint space: (ROS topic)  
  `/irb2600/joint_path_command` 
- Send spline in joint space: (ROS action)  
  `/irb2600/joint_trajectory_action` 

### IRB2600 With Moveit!
In terminal, run: 
```
roslaunch abb_irb2600_12_165_moveit_config moveit_planning_execution.launch sim:=false
```


## ABB IRB120
[ROS packages](./irb120/)
