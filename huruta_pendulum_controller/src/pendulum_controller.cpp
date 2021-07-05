#include <quadruped/FFPD_controller.hpp>
#include <iostream>
#include <unistd.h>

namespace ffpd_controller{
    FFPDController::FFPDController()
    {
    }

    bool FFPDController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n){
        
        // Get joint names from the parameter server
        std::string my_joint;
        FRjoint_.resize(3);
        if (!n.getParam("FRjoint1", my_joint)){
            ROS_ERROR("Could not find joint name");
            return false;
        }
        // get the joint object to use in the realtime loop
        FRjoint_[0] = hw->getHandle(my_joint);  // throws on failure
        if (!n.getParam("FRjoint2", my_joint)){
            ROS_ERROR("Could not find joint name");
            return false;
        }
        FRjoint_[1] = hw->getHandle(my_joint);  // throws on failure
        if (!n.getParam("FRjoint3", my_joint)){
            ROS_ERROR("Could not find joint name");
            return false;
        }
        FRjoint_[2] = hw->getHandle(my_joint);  // throws on failure

        FLjoint_.resize(3);
        if (!n.getParam("FLjoint1", my_joint)){
            ROS_ERROR("Could not find joint name");
            return false;
        }
        // get the joint object to use in the realtime loop
        FLjoint_[0] = hw->getHandle(my_joint);  // throws on failure
        if (!n.getParam("FLjoint2", my_joint)){
            ROS_ERROR("Could not find joint name");
            return false;
        }
        FLjoint_[1] = hw->getHandle(my_joint);  // throws on failure
        if (!n.getParam("FLjoint3", my_joint)){
            ROS_ERROR("Could not find joint name");
            return false;
        }
        FLjoint_[2] = hw->getHandle(my_joint);  // throws on failure

        RRjoint_.resize(3);
        if (!n.getParam("RRjoint1", my_joint)){
            ROS_ERROR("Could not find joint name");
            return false;
        }
        // get the joint object to use in the realtime loop
        RRjoint_[0] = hw->getHandle(my_joint);  // throws on failure
        if (!n.getParam("RRjoint2", my_joint)){
            ROS_ERROR("Could not find joint name");
            return false;
        }
        RRjoint_[1] = hw->getHandle(my_joint);  // throws on failure
        if (!n.getParam("RRjoint3", my_joint)){
            ROS_ERROR("Could not find joint name");
            return false;
        }
        RRjoint_[2] = hw->getHandle(my_joint);  // throws on failure

        RLjoint_.resize(3);
        if (!n.getParam("RLjoint1", my_joint)){
            ROS_ERROR("Could not find joint name");
            return false;
        }
        // get the joint object to use in the realtime loop
        RLjoint_[0] = hw->getHandle(my_joint);  // throws on failure
        if (!n.getParam("RLjoint2", my_joint)){
            ROS_ERROR("Could not find joint name");
            return false;
        }
        RLjoint_[1] = hw->getHandle(my_joint);  // throws on failure
        if (!n.getParam("RLjoint3", my_joint)){
            ROS_ERROR("Could not find joint name");
            return false;
        }
        RLjoint_[2] = hw->getHandle(my_joint);  // throws on failure

        // std::string sensor_name = "imu_sensor";
        // sensors_ = hw->getHandle(sensor_name);


        sub_FFPD_command = n.subscribe<quadruped_msgs::FFPDCommand>("ffpd_command", 1, &FFPDController::setCommand_CB, this, ros::TransportHints().udp());


        if(!kdl_parser::treeFromFile("/home/ubuntu/win10/catkin_ws/src/laikago_description/robot/robot_control.urdf", my_tree)){
           ROS_ERROR("Failed to construct kdl tree");
           return false;
        }

        ROS_INFO("my_tree was build %i\n", my_tree.getNrOfJoints());
        FR_Leg.reset(new LegObj(my_tree, "FR"));
        FL_Leg.reset(new LegObj(my_tree, "FL"));
        RR_Leg.reset(new LegObj(my_tree, "RR"));
        RL_Leg.reset(new LegObj(my_tree, "RL"));


        for(int i=0; i<12; i++) command.FFTorque[i] = 0;
        for(int i=0; i<12; i++) command.QDotdes[i] = 0;
        
        command.Qdes[3] = 0.0; command.Qdes[4]  = 0.0; command.Qdes[5]  = 0.0;
        command.Qdes[0] = 0.0; command.Qdes[1]  = 0.0; command.Qdes[2]  = 0.0;
        command.Qdes[6] = 0.0; command.Qdes[7]  = 0.5; command.Qdes[8]  = 0.0;
        command.Qdes[9] = 0.0; command.Qdes[10] = 0.5; command.Qdes[11] = 0.0;
        command_.writeFromNonRT(command);


        body_state_publisher.reset(
            new realtime_tools::RealtimePublisher<quadruped_msgs::BodyState>
            (n, "/state_raw", 1));

        return true;
    }

    void FFPDController::setCommand_CB(const quadruped_msgs::FFPDCommandConstPtr& msg)
    {
        for(int i=0; i < 12; i ++){
            command_CB.FFTorque[i] = msg->FFTorque[i];
            command_CB.Qdes[i] = msg->Qdes[i];
            command_CB.QDotdes[i] = msg->QDotdes[i];
        }
        command_.writeFromNonRT(command_CB);
    }


    void FFPDController::update(const ros::Time& time, const ros::Duration& period)
    {   
        // std::cout << "   update   \n";
        command = *(command_.readFromRT());

        int cnt=0;
        for(int i=0; i<12; i++){
            if(command.FFTorque[i]==0) cnt++;
        }
        if(cnt == 12)
        {
            Kp[0] = 900; Kp[1] = 900; Kp[2] = 900;
            Kd[0] = 10; Kd[1] = 10; Kd[2] = 10;
        }else{
            Kp[0] = 6; Kp[1] = 6; Kp[2] = 6;
            Kd[0] = 2; Kd[1] = 2; Kd[2] = 2;
        }

        FR_q << (float)(FRjoint_[0].getPosition()), (float)(FRjoint_[1].getPosition()), (float)(FRjoint_[2].getPosition());
        FR_q_dot << (float)(FRjoint_[0].getVelocity()), (float)(FRjoint_[1].getVelocity()), (float)(FRjoint_[2].getVelocity());
        FR_Leg->ComputePos(FR_q, FR_pos);
        for(int i=0; i<3; i++){
            torques[i+3] = command.FFTorque[i+FR_ID*3] + Kp[i]*(command.Qdes[i+FR_ID*3] - FR_q[i]) + Kd[i]*(command.QDotdes[i+FR_ID*3] - FR_q_dot[i]);
        }
        
        Eigen::Vector3f FL_torque = Eigen::Vector3f::Zero();
        FL_q << (float)(FLjoint_[0].getPosition()), (float)(FLjoint_[1].getPosition()), (float)(FLjoint_[2].getPosition());
        FL_q_dot << (float)(FLjoint_[0].getVelocity()), (float)(FLjoint_[1].getVelocity()), (float)(FLjoint_[2].getVelocity()); 
        FL_Leg->ComputePos(FL_q, FL_pos);
        for(int i=0; i<3; i++){
            torques[i] = command.FFTorque[i+FL_ID*3] + Kp[i]*(command.Qdes[i+FL_ID*3] - FL_q[i]) + Kd[i]*(command.QDotdes[i+FL_ID*3] - FL_q_dot[i]);
        }
        
        Eigen::Vector3f RR_torque = Eigen::Vector3f::Zero();
        RR_q << (float)(RRjoint_[0].getPosition()), (float)(RRjoint_[1].getPosition()), (float)(RRjoint_[2].getPosition());
        RR_q_dot << (float)(RRjoint_[0].getVelocity()), (float)(RRjoint_[1].getVelocity()), (float)(RRjoint_[2].getVelocity()); 
        RR_Leg->ComputePos(RR_q, RR_pos);
        for(int i=0; i<3; i++){
            torques[i+9] = command.FFTorque[i+RR_ID*3] + Kp[i]*(command.Qdes[i+RR_ID*3] - RR_q[i]) + Kd[i]*(command.QDotdes[i+RR_ID*3] - RR_q_dot[i]);
        }
        
        Eigen::Vector3f RL_torque = Eigen::Vector3f::Zero();
        RL_q << (float)(RLjoint_[0].getPosition()), (float)(RLjoint_[1].getPosition()), (float)(RLjoint_[2].getPosition());
        RL_q_dot << (float)(RLjoint_[0].getVelocity()), (float)(RLjoint_[1].getVelocity()), (float)(RLjoint_[2].getVelocity()); 
        RL_Leg->ComputePos(RL_q, RL_pos);
        for(int i=0; i<3; i++){
            torques[i+6] = command.FFTorque[i+RL_ID*3] + Kp[i]*(command.Qdes[i+RL_ID*3] - RL_q[i]) + Kd[i]*(command.QDotdes[i+RL_ID*3] - RL_q_dot[i]);
        }
        
        
        FLjoint_[0].setCommand(torques[0]);
        FLjoint_[1].setCommand(torques[1]);
        FLjoint_[2].setCommand(torques[2]);
        
        FRjoint_[0].setCommand(torques[3]);
        FRjoint_[1].setCommand(torques[4]);
        FRjoint_[2].setCommand(torques[5]);

        RLjoint_[0].setCommand(torques[6]);
        RLjoint_[1].setCommand(torques[7]);
        RLjoint_[2].setCommand(torques[8]);

        RRjoint_[0].setCommand(torques[9]);
        RRjoint_[1].setCommand(torques[10]);
        RRjoint_[2].setCommand(torques[11]);



        PublishState();

    }

    void FFPDController::PublishState(){
        for(int i=0; i<3; i++) {
            state.Leg_position[i+FR_ID*3] = FR_pos[i];
            state.Leg_position[i+FL_ID*3] = FL_pos[i];
            state.Leg_position[i+RR_ID*3] = RR_pos[i];
            state.Leg_position[i+RL_ID*3] = RL_pos[i];
        }

        for(int i=0; i<3; i++){
            state.Q[i+FL_ID * 3] = FL_q[i];
            state.Q[i+FR_ID * 3] = FR_q[i];
            state.Q[i+RL_ID * 3] = RL_q[i];
            state.Q[i+RR_ID * 3] = RR_q[i];

            state.QDot[i+FL_ID * 3] = FL_q_dot[i];
            state.QDot[i+FR_ID * 3] = FR_q_dot[i];
            state.QDot[i+RL_ID * 3] = RL_q_dot[i];
            state.QDot[i+RR_ID * 3] = RR_q_dot[i];

            state.torques[i+FL_ID * 3] = torques[i];
            state.torques[i+FR_ID * 3] = torques[i+3];
            state.torques[i+RL_ID * 3] = torques[i+6];
            state.torques[i+RR_ID * 3] = torques[i+9];
        }
        

        if(body_state_publisher && body_state_publisher->trylock())
        {
            body_state_publisher->msg_ = state;
            body_state_publisher->unlockAndPublish();
        }
    }

    void FFPDController::starting(const ros::Time& time){}
    void FFPDController::stopping(const ros::Time& time){}

PLUGINLIB_EXPORT_CLASS(ffpd_controller::FFPDController, controller_interface::ControllerBase);
}

