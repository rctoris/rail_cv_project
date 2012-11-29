/*!
 * \file robot.cpp
 * \brief Main robot controller for toy computer object recognition assignment.
 *
 * The robot moves the robot in response to the processor node. Additional information is available
 * in the project's writeup.
 *
 * \author Russell Toris, David Kent, Adrian‎ Boteanu
 * \date November 28, 2012
 */

#include <arm_navigation_msgs/SimplePoseConstraint.h>
#include <arm_navigation_msgs/utils.h>
#include <iostream>
#include <rail_cv_project/robot.h>
#include <rail_cv_project/processor.h>

using namespace std;

robot::robot() :
    move_l_arm("/move_left_arm", true), move_r_arm("/move_right_arm", true)
{
  last_x = -100;
  last_y = -100;
  last_z = -100;

  move_l_arm.waitForServer();
  move_r_arm.waitForServer();
  ROS_INFO("Arm control started");

  // reset the arms to begin
  reset_left();
  reset_right();
}

void robot::reset_right()
{
  // move the arm to the side
  move_arm_to_pose(0.095, -0.884, 1.144, BASE_FRAME, "right_arm");
}

void robot::reset_left()
{
  // move the arm to the side
  move_arm_to_pose(0.095, 0.884, 1.144, BASE_FRAME, "left_arm");
}

void robot::move_arm_to_pose(float x, float y, float z, string frame, string group)
{
  // initialize the goal
  arm_navigation_msgs::MoveArmGoal arm_goal;
  arm_goal.motion_plan_request.group_name = group;
  arm_goal.motion_plan_request.num_planning_attempts = 1;
  arm_goal.motion_plan_request.planner_id = "";
  arm_goal.planner_service_name = "ompl_planning/plan_kinematic_path";
  arm_goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  arm_navigation_msgs::SimplePoseConstraint pose;
  pose.header.frame_id = frame;
  // check for which link to move
  if (group.compare("right_arm") == 0)
    pose.link_name = "r_wrist_roll_link";
  else
    pose.link_name = "l_wrist_roll_link";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;

  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  // check for which angle to go it
  if (group.compare("right_arm") == 0)
    pose.pose.orientation.w = -1.0;
  else
    pose.pose.orientation.w = 1.0;

  // set the tolerances
  pose.absolute_position_tolerance.x = 0.02;
  pose.absolute_position_tolerance.y = 0.02;
  pose.absolute_position_tolerance.z = 0.02;
  pose.absolute_roll_tolerance = 0.04;
  pose.absolute_pitch_tolerance = 0.04;
  pose.absolute_yaw_tolerance = 0.04;
  arm_navigation_msgs::addGoalConstraintToMoveArmGoal(pose, arm_goal);

  // attempt to move
  ROS_INFO("Moving arm...");
  // set which arm we are using and send the goal
  if (group.compare("right_arm") == 0)
  {
    move_r_arm.sendGoal(arm_goal);
    if (move_r_arm.waitForResult(ros::Duration(60.0)))
      move_r_arm.cancelGoal();
  }
  else
  {
    move_l_arm.sendGoal(arm_goal);
    if (move_l_arm.waitForResult(ros::Duration(60.0)))
      move_l_arm.cancelGoal();
  }

  ROS_INFO("Arm move finished.");
}

void robot::checkTF()
{
  tf::StampedTransform stf;
  // we will wait up to one second to check the TF
  if (tfl.waitForTransform(CLOUD_FRAME, GOAL_FRAME, ros::Time(), ros::Duration(1.0)))
  {
    tfl.lookupTransform(CLOUD_FRAME, GOAL_FRAME, ros::Time(0), stf);
    // check if this is a newer one and it moved a fair amount
    if (stf.stamp_ > t)
    {
      // check if the object moved enough to justify a head move
      tf::Vector3 v = stf.getOrigin();
      double dist = sqrt(
          pow((double)(v.getX() - last_x), 2.0) + pow((double)(v.getY() - last_y), 2.0)
              + pow((double)(v.getZ() - last_z), 2.0));
      if (dist > HEAD_MOVE_THRESH)
      {
        move_arm_to_pose(v.getX(), v.getY(), v.getZ(), CLOUD_FRAME, "left_arm");
        reset_left();

        t = stf.stamp_;
        last_x = v.getX();
        last_y = v.getY();
        last_z = v.getZ();
      }
    }
  }
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "robot");
  robot r;

  // continue until a ctrl-c has occurred
  while (ros::ok())
  {
    // run an iteration of the TF checker
    r.checkTF();
    ros::spinOnce();
  }

  return EXIT_SUCCESS;
}
