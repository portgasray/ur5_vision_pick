### launch moveit 

```
roslaunch ur5_vision_pick_moveit_config demo.launch
```

### vision part
1. after run ur5_vision.py the center point of block will be publish by topic `camera_xy`

```
$ rostopic list

/camera_xy

```

2. use `rostopic echo /camera_xy`

you will see the publish contents

```
---
x: 320.0
y: 247.0
z: 0.0
---
x: 320.0
y: 247.0
z: 0.0
---
... ...

```
Q: z axis is none!

3. run a script
```
rosrun ur5_grasping ur5_vision.py
```

4.
```
$ static_transform_publisher -0.07427868848074914 -0.5151402712869437 0.5263202824952005 -0.3811774134284603 -0.7688461277047242 0.32927060898269445  0.39390389369185613 base_link camera_link
```