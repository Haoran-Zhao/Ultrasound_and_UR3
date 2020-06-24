#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "jog_msgs/JogJoint.h"
#include "ros/ros.h"
#include <ros/console.h>
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int32.h"

namespace to_twist
{
class UR3Xbox
{
public:
  UR3Xbox() : spinner_(1)
  {
    joy_sub_ = n_.subscribe("joy", 1, &UR3Xbox::joyCallback, this);
    mat_sub_ = n_.subscribe("goal_pos",1, &UR3Xbox::matCallback, this);
    ft_orien_sub_ = n_.subscribe("ft_orien",1, &UR3Xbox::ftOrienCallback, this);
    track_pub_ = n_.advertise<std_msgs::Int32>("track_flag", 1);
    twist_pub_ = n_.advertise<geometry_msgs::TwistStamped>("jog_arm_server/delta_jog_cmds", 1);
    joint_delta_pub_ = n_.advertise<jog_msgs::JogJoint>("jog_arm_server/joint_delta_jog_cmds", 1);
    spinner_.start();
    ros::waitForShutdown();
  };

private:
  ros::NodeHandle n_;
  ros::Subscriber joy_sub_, mat_sub_, ft_sub_,ft_orien_sub_;
  ros::Publisher twist_pub_, joint_delta_pub_, track_pub_;
  ros::AsyncSpinner spinner_;
  geometry_msgs::Twist ft_twist_;

  void ftOrienCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    ft_twist_.linear.x = msg->linear.x;
    ft_twist_.linear.y = msg->linear.y;
    ft_twist_.linear.z = msg->linear.z;
    ft_twist_.angular.x = msg->angular.x;
    ft_twist_.angular.y = msg->angular.y;
    ft_twist_.angular.z = msg->angular.z;
  }
  

  // Convert incoming joy commands to TwistStamped commands for jogging
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
  {
    // Cartesian jogging
    geometry_msgs::TwistStamped twist;
    twist.header.stamp = ros::Time::now();
    // This button is binary
    twist.twist.linear.x = -msg->buttons[4] + msg->buttons[5];
    // Double buttons
    twist.twist.linear.y = msg->axes[0]; // axis[0]
    twist.twist.linear.z = msg->axes[1]; // axis[1]
    twist.twist.angular.x = -msg->axes[3]; // axis[3]
    twist.twist.angular.y = msg->axes[4]; // axis[4]
    // A binary button
    twist.twist.angular.z = -msg->buttons[0] + msg->buttons[1];

    // Joint jogging
    jog_msgs::JogJoint joint_deltas;
    // This example is for a Motoman SIA5. joint_s is the base joint.
    joint_deltas.joint_names.push_back("shoulder_pan_joint");
    // Button 6: positive
    // Button 7: negative
    joint_deltas.deltas.push_back(msg->buttons[6] - msg->buttons[7]);
    joint_deltas.header.stamp = ros::Time::now();

    twist_pub_.publish(twist);
    joint_delta_pub_.publish(joint_deltas);

    std_msgs::Int32 track_flag;
    track_flag.data = msg->axes[2] - msg->axes[5];
    track_pub_.publish(track_flag);

  }

  void matCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    // Cartesian jogging
    geometry_msgs::TwistStamped twist;
    twist.header.stamp = ros::Time::now();
    // This button is binary
    twist.twist.linear.x = msg->linear.x;
    // Double buttons
    twist.twist.linear.y = msg->linear.y;
    twist.twist.linear.z = msg->linear.z;
    twist.twist.angular.x = -msg->angular.x;
    twist.twist.angular.y = msg->angular.y;
    // A binary button
    twist.twist.angular.z = -msg->angular.z;

    twist_pub_.publish(twist);
  }

};
}  // end to_twist namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xbox_to_twist");

  to_twist::UR3Xbox to_twist;

  return 0;
}
