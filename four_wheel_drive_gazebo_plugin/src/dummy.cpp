#include "four_wheel_drive_gazebo_plugin.h"

namespace gazebo {

  // Defining the Wheel Joints
  enum {
    FRONT_LEFT,
    FRONT_RIGHT,
    REAR_LEFT,
    REAR_RIGHT
  };

  // Constructor
  FourWheelDriveMultiWheel::FourWheelDriveMultiWheel() {}

  // Destructor
  FourWheelDriveMultiWheel::~FourWheelDriveMultiWheel() {
    delete rosnode_;
    delete transform_broadcaster_;
   }

 // Load the controller
  void FourWheelDriveMultiWheel::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

    this->parent = _parent;
    this->world = _parent->GetWorld();

    this->robot_namespace_ = "thorvald_ii";
    if (!_sdf->HasElement("robotNamespace")) {
      ROS_INFO("FourWheelDriveMultiWheel Plugin missing <robotNamespace>, defaults to \"%s\"", this->robot_namespace_.c_str());
    } else {
      this->robot_namespace_ = 
        _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    }

    //this->left_joint_names_ = "left_joint";
    if (!_sdf->HasElement("steering2")) {
      gzthrow("Have to specify space separated left side joint names via <leftJoints> tag!");
    } else {
      std::string joint_string = _sdf->GetElement("steering2")->Get<std::string>();
      boost::split( joint_names_[FRONT_LEFT], joint_string, boost::is_any_of(" ") );
    }

    //this->right_joint_names_ = "right_joint";
    if (!_sdf->HasElement("steering1")) {
      gzthrow("Have to specify space separated right side joint names via <rightJoints> tag!");
    } else {
      std::string joint_string = _sdf->GetElement("steering1")->Get<std::string>();
      boost::split( joint_names_[FRONT_RIGHT], joint_string, boost::is_any_of(" ") );
    }

    //this->left_joint_names_ = "left_joint";
    if (!_sdf->HasElement("steering3")) {
      gzthrow("Have to specify space separated left side joint names via <leftJoints> tag!");
    } else {
      std::string joint_string = _sdf->GetElement("steering3")->Get<std::string>();
      boost::split( joint_names_[REAR_LEFT], joint_string, boost::is_any_of(" ") );
    }

    //this->right_joint_names_ = "right_joint";
    if (!_sdf->HasElement("steering0")) {
      gzthrow("Have to specify space separated right side joint names via <rightJoints> tag!");
    } else {
      std::string joint_string = _sdf->GetElement("steering0")->Get<std::string>();
      boost::split( joint_names_[REAR_RIGHT], joint_string, boost::is_any_of(" ") );
    }

    this->platform_width = 0.570;
    if (!_sdf->HasElement("platform_width")) {
      ROS_WARN("FourWheelDriveMultiWheel Plugin (ns = %s) missing <platform_width>, defaults to %f",
          this->robot_namespace_.c_str(), this->platform_width);
    } else {
      this->platform_width = _sdf->GetElement("platform_width")->Get<double>();
    }

    this->wheel_radius = 0.400;
    if (!_sdf->HasElement("wheel_radius")) {
      ROS_WARN("FourWheelDriveMultiWheel Plugin (ns = %s) missing <wheel_radius>, defaults to %f",
          this->robot_namespace_.c_str(), this->wheel_radius);
    } else {
      this->wheel_radius = _sdf->GetElement("wheel_radius")->Get<double>();
    }

    this->torque = 5.0;
    if (!_sdf->HasElement("torque")) {
      ROS_WARN("FourWheelDriveMultiWheel Plugin (ns = %s) missing <torque>, defaults to %f",
          this->robot_namespace_.c_str(), this->torque);
    } else {
      this->torque = _sdf->GetElement("torque")->Get<double>();
    }

    this->command_topic_ = "joint_command";
    if (!_sdf->HasElement("commandTopic")) {
      ROS_WARN("FourWheelDriveMultiWheel Plugin (ns = %s) missing <commandTopic>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->command_topic_.c_str());
    } else {
      this->command_topic_ = _sdf->GetElement("commandTopic")->Get<std::string>();
    }

    this->odometry_topic_ = "odom";
    if (!_sdf->HasElement("odometryTopic")) {
      ROS_WARN("FourWheelDriveMultiWheel Plugin (ns = %s) missing <odometryTopic>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->odometry_topic_.c_str());
    } else {
      this->odometry_topic_ = _sdf->GetElement("odometryTopic")->Get<std::string>();
    }

    this->odometry_frame_ = "odom";
    if (!_sdf->HasElement("odometryFrame")) {
      ROS_WARN("FourWheelDriveMultiWheel Plugin (ns = %s) missing <odometryFrame>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->odometry_frame_.c_str());
    } else {
      this->odometry_frame_ = _sdf->GetElement("odometryFrame")->Get<std::string>();
    }

    this->robot_base_frame_ = "base_link";
    if (!_sdf->HasElement("robotBaseFrame")) {
      ROS_WARN("FourWheelDriveMultiWheel Plugin (ns = %s) missing <robotBaseFrame>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->robot_base_frame_.c_str());
    } else {
      this->robot_base_frame_ = _sdf->GetElement("robotBaseFrame")->Get<std::string>();
    }

    this->update_rate_ = 100.0;
    if (!_sdf->HasElement("updateRate")) {
      ROS_WARN("FourWheelDriveMultiWheel Plugin (ns = %s) missing <updateRate>, defaults to %f",
          this->robot_namespace_.c_str(), this->update_rate_);
    } else {
      this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();
    }


    this->publish_odometry_tf_ = true;
    if (!_sdf->HasElement("publishOdometryTf")) {
      ROS_WARN("FourWheelDriveMultiWheel Plugin (ns = %s) missing <publishOdometryTf>, defaults to %s",
               this->robot_namespace_.c_str(), this->publish_odometry_tf_ ? "true" : "false");
    } else {
      this->publish_odometry_tf_ = _sdf->GetElement("publishOdometryTf")->Get<bool>();
    }

    this->publish_odometry_msg_ = true;
    if (!_sdf->HasElement("publishOdometryMsg")) {
      ROS_WARN("FourWheelDriveMultiWheel Plugin (ns = %s) missing <publishOdometryMsg>, defaults to %s",
               this->robot_namespace_.c_str(), this->publish_odometry_msg_ ? "true" : "false");
    } else {
      this->publish_odometry_msg_ = _sdf->GetElement("publishOdometryMsg")->Get<bool>();
}

    // Initialize update rate stuff
    if (this->update_rate_ > 0.0) {
      this->update_period_ = 1.0 / this->update_rate_;
    } else {
      this->update_period_ = 0.0;
    }
    
    last_update_time_ = this->world->GetSimTime();

    // Initialize velocity stuff
    wheel_speed_[FRONT_RIGHT] = 0;
    wheel_speed_[FRONT_LEFT] = 0;
    wheel_speed_[REAR_RIGHT] = 0;
    wheel_speed_[REAR_LEFT] = 0;

    for (size_t side = 0; side < 4; ++side){
      for (size_t i = 0; i < joint_names_[side].size(); ++i){
        joints_[side].push_back(this->parent->GetJoint(joint_names_[side][i]));
        if (!joints_[side][i]){
          char error[200];
          snprintf(error, 200,
                   "FourWheelDriveMultiWheel Plugin (ns = %s) couldn't get hinge joint named \"%s\"",
                   this->robot_namespace_.c_str(), joint_names_[side][i].c_str());
          gzthrow(error);
        }
        // joints_[side][i]->SetMaxForce(0, torque);
      }
}

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. " << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

  rosnode_ = new ros::NodeHandle(this->robot_namespace_);

    // ROS: Subscribe to the joint command topic 
  ros::SubscribeOptions so = ros::SubscribeOptions::create<thorvald_ii_control::BaseState>("joint_command", 1, boost::bind(&FourWheelDriveMultiWheel::jointcmdVelCallback, this, _1), ros::VoidPtr(), &queue_);

   joint_command_subscriber_ = rosnode_->subscribe(so);
}

 // Update the controller
  void FourWheelDriveMultiWheel::UpdateChild() {
    common::Time current_time = this->world->GetSimTime();
    double seconds_since_last_update = 
      (current_time - last_update_time_).Double();
    if (seconds_since_last_update > update_period_) {

      if (this->publish_odometry_tf_ || this->publish_odometry_msg_){
        publishOdometry(seconds_since_last_update);
      }

      // Update robot in case new velocities have been requested
      getWheelVelocities();
      //joints[LEFT]->SetVelocity(0, wheel_speed_[LEFT] / wheel_diameter_);
      //joints[RIGHT]->SetVelocity(0, wheel_speed_[RIGHT] / wheel_diameter_);

      for (size_t side = 0; side < 4; ++side){
        for (size_t i = 0; i < joints_[side].size(); ++i){
          joints_[side][i]->SetVelocity(0, wheel_speed_[side]);
        }
      }

      last_update_time_+= common::Time(update_period_);
    }
}

  // Obtaining the wheel Velocities
  void FourWheelDriveMultiWheel::getWheelVelocities() {
    boost::mutex::scoped_lock scoped_lock(lock);

    wheel_speed_[FRONT_LEFT] = joint_command_.vel[0];
    wheel_speed_[FRONT_RIGHT] = joint_command_.vel[1];
    wheel_speed_[REAR_LEFT] = joint_command_.vel[2];
    wheel_speed_[REAR_RIGHT] = joint_command_.vel[3];
  }

  // Finalize the controller
  void FourWheelDriveMultiWheel::FiniChild() {
    alive_ = false;
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();
}

 void FourWheelDriveMultiWheel::QueueThread() {
    static const double timeout = 0.01;

    while (alive_ && rosnode_->ok()) {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
}

  void FourWheelDriveMultiWheel::jointcmdVelCallback(const thorvald_ii_control::BaseState::ConstPtr& joint_cmd_msg) {

    // boost::mutex::scoped_lock scoped_lock(lock);
    joint_command_.drive_mode = joint_cmd_msg->drive_mode;
    joint_command_.vel = joint_cmd_msg->vel;
    joint_command_.position = joint_cmd_msg->position;
    joint_command_.pos_max_vel = joint_cmd_msg->pos_max_vel;
  }

  // Obtaing and broadcasting Odometry
  void FourWheelDriveMultiWheel::publishOdometry(double step_time) {
    ros::Time current_time = ros::Time::now();
    std::string odom_frame = 
      tf::resolve(tf_prefix_, odometry_frame_);
    std::string base_footprint_frame = 
      tf::resolve(tf_prefix_, robot_base_frame_);

    // getting data for base_footprint to odom transform
    math::Pose pose = this->parent->GetWorldPose();

    tf::Quaternion qt(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
    tf::Vector3 vt(pose.pos.x, pose.pos.y, pose.pos.z);

    tf::Transform base_footprint_to_odom(qt, vt);

    if (this->publish_odometry_tf_){
      transform_broadcaster_->sendTransform(
            tf::StampedTransform(base_footprint_to_odom, current_time,
                                 odom_frame, base_footprint_frame));
    }

    // publish odom topic
    odom_.pose.pose.position.x = pose.pos.x;
    odom_.pose.pose.position.y = pose.pos.y;

    odom_.pose.pose.orientation.x = pose.rot.x;
    odom_.pose.pose.orientation.y = pose.rot.y;
    odom_.pose.pose.orientation.z = pose.rot.z;
    odom_.pose.pose.orientation.w = pose.rot.w;
    odom_.pose.covariance[0] = 0.00001;
    odom_.pose.covariance[7] = 0.00001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = 0.001;

    // get velocity in /odom frame
    math::Vector3 linear;
    linear = this->parent->GetWorldLinearVel();
    odom_.twist.twist.angular.z = this->parent->GetWorldAngularVel().z;

    // convert velocity to child_frame_id (aka base_footprint)
    float yaw = pose.rot.GetYaw();
    odom_.twist.twist.linear.x = cosf(yaw) * linear.x + sinf(yaw) * linear.y;
    odom_.twist.twist.linear.y = cosf(yaw) * linear.y - sinf(yaw) * linear.x;

    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    if (this->publish_odometry_msg_){
      odometry_publisher_.publish(odom_);
    }
  }

 GZ_REGISTER_MODEL_PLUGIN(FourWheelDriveMultiWheel)
}
