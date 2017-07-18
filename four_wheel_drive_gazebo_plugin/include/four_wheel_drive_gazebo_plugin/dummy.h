#ifndef FOUR_WHEEL_DRIVE_GAZEBO_PLUGIN_H
#define FOUR_WHEEL_DRIVE_GAZEBO_PLUGIN_H

#include <string>
#include <map>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <sstream>
#include <stdio.h>

// Gazebo
#include <sdf/sdf.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_camera_utils.h>
#include <gazebo/math/gzmath.hh>
#include <gazebo/physics/Model.hh>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <thorvald_ii_control/BaseState.h>

namespace gazebo
{

 class FourWheelDriveMultiWheel:public ModelPlugin {

 public:
   FourWheelDriveMultiWheel();
   ~FourWheelDriveMultiWheel();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

 protected:
      virtual void UpdateChild();
      virtual void FiniChild();

 private:
     void publishOdometry(double step_time);
     void getWheelVelocities();

     std::vector<std::string> joint_names_[4];
     std::vector<physics::JointPtr> joints_[4];

     physics::WorldPtr world;
     physics::ModelPtr parent;
     event::ConnectionPtr update_connection_;

      double platform_width;
      double wheel_radius;
      double torque;
      double wheel_speed_[4];

      // ROS STUFF
      ros::NodeHandle* rosnode_;
      ros::Publisher odometry_publisher_;
      ros::Subscriber joint_command_subscriber_;
      tf::TransformBroadcaster *transform_broadcaster_;
      nav_msgs::Odometry odom_; 
      bool alive_;
      std::string tf_prefix_;
      thorvald_ii_control::BaseState joint_command_;

      boost::mutex lock;

      std::string robot_namespace_;
      std::string command_topic_;
      std::string odometry_topic_;
      std::string odometry_frame_;
      std::string robot_base_frame_;

      // Custom Callback Queue
      ros::CallbackQueue queue_;
      boost::thread callback_queue_thread_;
      void QueueThread();

     // DiffDrive stuff
     void jointcmdVelCallback(const thorvald_ii_control::BaseState::ConstPtr& joint_cmd_msg);

      // Update Rate
      double update_rate_;
      double update_period_;
      common::Time last_update_time_;

      bool publish_odometry_tf_;
      bool publish_odometry_msg_;
 };
}

#endif
