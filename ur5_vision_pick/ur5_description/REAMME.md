1. convert xacro to urdf
```
./convert_xacro_urdf.sh src/ur5_vision_pick/ur5_description/urdf/
```

2. revise the urdf `ur5_hitbot_with_ilc_platform.xacro.urdf`


3. check verification

```
urdf_to_graphiz ur5_hitbot_with_ilc_platform.xacro.urdf
```

if success, two files will generate!

```
Created file ur5_hitbot_with_ilc_platform.gv
Created file ur5_hitbot_with_ilc_platform.pdf
```

when error occurs
- Error:   Failed to build tree: Joint [platform_base_robot_install_plate_joint] is missing a parent and/or child link specification.
         at line 226 in /build/urdfdom-UJ3kd6/urdfdom-0.4.1/urdf_parser/src/model.cpp
ERROR: Model Parsing the xml failed

4. add a launch file include the changes

```
<launch>
<arg name="gui" default="True" />
  
    <param name="robot_description" 
            command="$(find xacro)/xacro --inorder '$(find ur5_description)/urdf/ur5_hitbot_with_ilc_platform.xacro.urdf'"/>

    <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />

    <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher" />

    <node name="rviz" pkg="rviz" type="rviz" args="-d $(find ur5_description)/config/ur5_view.rviz" required="true" />
</launch>

```

5. lauch and view the reslut

```
roslaunch ur5_description display.launch
```
or
```
roslaunch urdf_tutorial display.launch model:=/home/relaybot/ROS_tutorial/src/urdf_tutorial-kinetic/urdf/xxxx.urdf
```
