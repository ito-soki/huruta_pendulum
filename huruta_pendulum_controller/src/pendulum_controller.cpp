#include <pendulum_control/pendulum_controller.hpp>
#include <iostream>
#include <unistd.h>

namespace pendulum_controller{
    PendulumController::PendulumController()
    {
    }

    bool PendulumController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n){
        
        // Get joint names from the parameter server
        joints.resize(2);
        
        joints[0] = hw->getHandle("motor_joint");  // throws on failure

        joints[1] = hw->getHandle("free_joint");  // throws on failure


        sub_Pendulum_command = n.subscribe<std_msgs::Bool>("pendulum_command", 1, &PendulumController::setCommand_CB, this, ros::TransportHints().udp());



        command = true;
        command_CB = true;
        command_.writeFromNonRT(command);
        // Kp[0] = 0.5; Kp[1] = 3.0;
        // Kd[0] = 0.02; Kd[1] = 0.1;
        F[0] = 0.150354; F[1] = -1.3752295;
        F[2] = 0.1098012; F[3] = -0.2257347;
        return true;
    }

    void PendulumController::setCommand_CB(const std_msgs::BoolConstPtr& msg)
    {
        command_CB = msg->data;
        command_.writeFromNonRT(command_CB);
    }


    void PendulumController::update(const ros::Time& time, const ros::Duration& period)
    {   
        // std::cout << "   update   \n";
        command = *(command_.readFromRT());

        float q[2];
        float q_dot[2];
        q[0] = (float)(joints[0].getPosition()); q[1] = (float)(joints[1].getPosition());
        q_dot[0] = (float)(joints[0].getVelocity()); q_dot[1] = (float)(joints[1].getVelocity());
        float x[4];
        x[0] = q[0]; x[1] = q[1];
        x[2] = q_dot[0]; x[3] = q_dot[1];
        
        //torque = Kp[0]*q[0] + Kd[0]*q_dot[0] -Kp[1]*q[1] - Kd[1]*q_dot[1];
        torque = 0;
        for (int i=0; i<4; i++){
            torque += F[i] * x[i];
        }
        
        
        joints[0].setCommand(torque);
        joints[1].setCommand(0.0);

    }

    void PendulumController::starting(const ros::Time& time){}
    void PendulumController::stopping(const ros::Time& time){}

PLUGINLIB_EXPORT_CLASS(pendulum_controller::PendulumController, controller_interface::ControllerBase);
}

