# Introduction


A showcase of the project, transfer to 👉 https://portgasray.github.io/project/robot-grasp/

![](https://portgasray.github.io/project/robot-grasp/featured_hue20720df3ff6099eca58a7f5f9fa450e_1614900_720x0_resize_q90_lanczos.jpg)



## Vision 

**Algorithm.** The Detection Algorithm is inherited the Benckmakr of Andreas ten Pas, named [Grasp Pose Detection](https://github.com/atenpas/gpd) (GPD). GPD used a geometric way to generated the candidates of the grasp hypothesis. 

**Platform.** The OpenViNO Toolkit from Intel. The **RealSense** camera is supported by this toolkit.


The Whole part of the vision demo can found at [ros2_grasp_library](https://github.com/intel/ros2_grasp_library) with the latest version.


## Manipulation

**Hand-eye calibration.** Although, you could find the tool of the calibration from [Intel Corporation](https://github.com/intel), we still encourage you to explore the basic one which start from our former steps. 👉 [UR5 REALSENSE CALIBRATION](https://github.com/portgasray/ur5_realsense_calibration)
> Here is the promotion!!! the CALIBRATION repository is tested by serveral experiments with the group members. The document well summarized the processing of the calibration, which is easy to used and plugin.

**Moveit.** The motion planing is hosting by the 3rd party.



