#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <pr2_hardware_interface/hardware_interface.h>


namespace four_wheel_drive_controller_ns{

//Inheriting Controller class inside pr2_controller_interface
class FourWheelDriveControllerClass: public pr2_controller_interface::Controller
{
private:
pr2_mechanism_model::JointState* joint_state_;
double init_pos_;
public:
virtual bool init(pr2_mechanism_model::RobotState *robot,
ros::NodeHandle &n);
virtual void starting();
virtual void update();
virtual void stopping();
};


}
