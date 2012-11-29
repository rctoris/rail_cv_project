/*!
 * \file robot.h
 * \brief Main robot controller for toy computer object recognition assignment.
 *
 * The robot moves the robot in response to the processor node. Additional information is available
 * in the project's writeup.
 *
 * \author Russell Toris, David Kent, Adrian‎ Boteanu
 * \date November 28, 2012
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <arm_navigation_msgs/MoveArmAction.h>

/*!
 * \def HEAD_MOVE_THRESH
 * Distance threshold in meters used when deciding to move the head.
 */
#define HEAD_MOVE_THRESH 0.01

/*!
 * \class robot
 * \brief Main robot control object.
 *
 * The robot class controls the robot in response to the object recognition.
 */
class robot
{
public:
  /**
   * Creates a robot. This will listen to changes in the TF tree.
   */
  robot();

  /**
   * Checks the TF tree for a new object position.
   */
  void checkTF();

  /**
   * Reset the left arm to its start position.
   */
  void reset_left();

  /**
   * Reset the right arm to its start position.
   */
  void reset_right();

private:
  /**
   * Move the given arm to the given pose based on the given reference frame.
   *
   * @param x the X position in meters
   * @param y the Y position in meters
   * @param z the Z position in meters
   * @param frame the reference to move the arm with respect to
   * @param group the group name for the arm joints
   */
  void move_arm_to_pose(float x, float y, float z, std::string frame, std::string group);

  ros::NodeHandle node; /*!< main ROS node handle */

  ros::Time t; /*!< used to keep track of the last TF seen */

  tf::TransformListener tfl; /*!< used to keep track of the object in 3D space */
  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_l_arm, move_r_arm; /*!< used to move the robot's arms */

  float last_x, last_y, last_z; /*!< last known location of the object */
};

/*!
 * Creates and runs the robot node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv);

#endif
