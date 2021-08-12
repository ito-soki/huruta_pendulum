#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <pluginlib/class_list_macros.h>
#include <boost/scoped_ptr.hpp>
#include <std_msgs/Bool.h>

namespace pendulum_controller{

    class PendulumController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
        public:
            PendulumController();
            bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &controller_nh);
            void update(const ros::Time& time, const ros::Duration& period);
            void starting(const ros::Time& time);
            void stopping(const ros::Time& time);
            void setCommand_CB(const std_msgs::BoolConstPtr& msg);
            
        private:
            std::vector<hardware_interface::JointHandle> joints;
            // hardware_interface::ImuSensorHandle sensors_;
            ros::Subscriber sub_Pendulum_command;
            realtime_tools::RealtimeBuffer<bool> command_;
            bool command;
            bool command_CB;

            float F[4];
            float torque;
    };

}