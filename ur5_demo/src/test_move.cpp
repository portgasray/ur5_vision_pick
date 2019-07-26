// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ur5_demo");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("arm");
  group.setPlanningTime(45.0);

  //获取终端link的名称
  std::string end_effector_link = group.getEndEffectorLink();

  //设置目标位置所使用的参考坐标系
  std::string reference_frame = "base_link";
  group.setPoseReferenceFrame(reference_frame);

  //当运动规划失败后，允许重新规划
  group.allowReplanning(true);

  //设置位置(单位：米)和姿态（单位：弧度）的允许误差
  group.setGoalPositionTolerance(0.001);
  group.setGoalOrientationTolerance(0.01);

  //设置允许的最大速度和加速度
  group.setMaxAccelerationScalingFactor(0.2);
  group.setMaxVelocityScalingFactor(0.2);

  // addCollisionObjects(planning_scene_interface);

  // Move to the home pose
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  group.setStartStateToCurrentState();
  group.setNamedTarget("home");
  
  if (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    if (group.execute(my_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
      return 1;
  }

  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();  
  
  // // 设置机器人终端的目标位置
  //   geometry_msgs::Pose target_pose;
  //   target_pose.orientation.x = 0.70692;
  //   target_pose.orientation.y = 0.0;
  //   target_pose.orientation.z = 0.0;
  //   target_pose.orientation.w = 0.70729;

  //   target_pose.position.x = 0.2593;
  //   target_pose.position.y = 0.0636;
  //   target_pose.position.z = 0.1787;

  //   // 设置机器臂当前的状态作为运动初始状态
  //   group.setStartStateToCurrentState();

  //   group.setPoseTarget(target_pose);

  //   // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
  //   moveit::planning_interface::MoveGroupInterface::Plan plan;
  //   moveit::planning_interface::MoveItErrorCode success = arm.plan(plan);

  //   ROS_INFO("Plan (pose goal) %s",success?"":"FAILED");   

  //   //让机械臂按照规划的轨迹开始运动。
  //   if(success)
  //     group.execute(plan);
  //   sleep(1);

  //   // 控制机械臂先回到初始化位置
  //   group.setNamedTarget("home");
  //   group.move();
  //   sleep(1);


  // pick(group);

  // ros::WallDuration(1.0).sleep();

  // place(group);

  ros::waitForShutdown();
  return 0;
}