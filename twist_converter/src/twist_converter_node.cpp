#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "thorvald_ii_control/ThorvaldTwist.h"

class TwistConverterNode
{
  public:
  TwistConverterNode();    

  private:
  ros::NodeHandle nh_;
  ros::Subscriber twist_sub_;
  ros::Publisher twist_pub_;

  void twistCallback(const geometry_msgs::Twist::ConstPtr& input_twist);


};

TwistConverterNode::TwistConverterNode()
{
  twist_sub_ = nh_.subscribe("cmd_vel_twist", 1, &TwistConverterNode::twistCallback, this);
  twist_pub_ = nh_.advertise<thorvald_ii_control::ThorvaldTwist>("cmd_vel", 1);
}


void TwistConverterNode::twistCallback(const geometry_msgs::Twist::ConstPtr& input_twist)
{
  
  thorvald_ii_control::ThorvaldTwist output_twist;
  output_twist.drive_mode = 0;
  output_twist.twist.twist.twist = *input_twist;
  //output_twist.twist.twist.twist.angular.z = -output_twist.twist.twist.twist.angular.z;
  twist_pub_.publish(output_twist);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "twist_converter_node");
  TwistConverterNode twist_converter;
  ros::spin();
}


