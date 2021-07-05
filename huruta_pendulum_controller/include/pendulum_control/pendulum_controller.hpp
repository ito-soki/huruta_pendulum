#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <pluginlib/class_list_macros.h>
#include <quadruped/leg_kinematics.hpp>
#include <boost/scoped_ptr.hpp>
#include <quadruped_msgs/FFPDCommand.h>
#include <quadruped_msgs/BodyState.h>
#include <quadruped/CppTypes.h>

namespace ffpd_controller{

    class FFPDController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
        public:
            FFPDController();
            bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &controller_nh);
            void update(const ros::Time& time, const ros::Duration& period);
            void starting(const ros::Time& time);
            void stopping(const ros::Time& time);
            void setCommand_CB(const quadruped_msgs::FFPDCommandConstPtr& msg);
            
        private:
            std::vector<hardware_interface::JointHandle> FRjoint_;
            std::vector<hardware_interface::JointHandle> FLjoint_;
            std::vector<hardware_interface::JointHandle> RRjoint_;
            std::vector<hardware_interface::JointHandle> RLjoint_;
            // hardware_interface::ImuSensorHandle sensors_;
            ros::Subscriber sub_FFPD_command;
            realtime_tools::RealtimeBuffer<FFPDCommand> command_;
            FFPDCommand command;
            FFPDCommand command_CB;

            KDL::Tree my_tree;

            boost::scoped_ptr<LegObj> FR_Leg;
            boost::scoped_ptr<LegObj> FL_Leg;
            boost::scoped_ptr<LegObj> RR_Leg;
            boost::scoped_ptr<LegObj> RL_Leg;

            Eigen::Vector3f FR_pos, FL_pos, RR_pos, RL_pos;
            Eigen::Vector3f FR_q, FL_q, RR_q, RL_q;
            Eigen::Vector3f FR_q_dot, FL_q_dot, RR_q_dot, RL_q_dot;

            void PublishState();
            std::unique_ptr<
                realtime_tools::RealtimePublisher<
                quadruped_msgs::BodyState> > body_state_publisher;
            quadruped_msgs::BodyState state;

            float Kp[3], Kd[3];
            float torques[12];
    };

    inline void check_value(const Eigen::Vector3f& torque)
    {
        if(torque[0] > 20 || torque[0] < -20) ROS_INFO("hip joint torque was out of range...");
        if(torque[1] > 55 || torque[1] < -55) ROS_INFO("thigh joint torque was out of range...");
        if(torque[2] > 55 || torque[2] < -55) ROS_INFO("calf joint torque was out of range...");
    }
}