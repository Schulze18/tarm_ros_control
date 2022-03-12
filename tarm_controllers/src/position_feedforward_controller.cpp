#include <tarm_controllers/position_feedforward_controller.h>


namespace tarm_controllers{

    PositionFeedforwardEffortController::PositionFeedforwardEffortController(){

    }

    bool PositionFeedforwardEffortController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &nh){

        // Get Yaml parameters
        XmlRpc::XmlRpcValue joints_name, pid_gains;
        // nh.getParam("joints", joints_name);
        // nh.getParam("pid", pid_gains);

        if (!nh.getParam("joints", joints_name)){
            ROS_ERROR("Empty Joints List");
            return false;
        }
        if (!nh.getParam("pid", pid_gains)){
            ROS_ERROR("Empty PID Gains");
            return false;
        }

        ROS_WARN("Number of joints %d", joints_name.size());

        if (num_joints < joints_name.size()){
            ROS_ERROR("Missing joints name");
            return false;
        }
        else if (num_joints > joints_name.size()){
            ROS_ERROR("Defined names excced limit of %d", joints_name.size());
            return false;
        }

        // Resize data
        pid_controller.resize(num_joints);
        joint_feedforward_torques.resize(num_joints);
        commanded_effort.resize(num_joints);
        joint_position.resize(num_joints);
        joint_velocity.resize(num_joints);
        joint_acc.resize(num_joints);
        joint_reference_position.resize(num_joints);
        joint_reference_velocity.resize(num_joints);
        joint_reference_acc.resize(num_joints);
        error.resize(num_joints);
        error_old.resize(num_joints);
        error_dot.resize(num_joints);
        joints_hw.resize(num_joints);
        for (int i = 0; i < num_joints; i++){
            joints_hw[i] = hw->getHandle(joints_name[i]);
            pid_controller[i].initPid(pid_gains["p"], pid_gains["i"], pid_gains["d"], 10000, -10000); 
        }
        
        command_subscriber = nh.subscribe<std_msgs::Float64MultiArray>("command", 1, &PositionFeedforwardEffortController::commandCallback, this);

        // joint_reference_position[0] = 0.2;
        // joint_reference_position[1] = 0.2; 
        for (int i = 0; i < num_joints; i++){
            joint_reference_position[i] = 0;
            joint_reference_velocity[i] = 0;
            joint_reference_acc[i] = 0;
        }

        // Parameters
        m1 = 1.0;
        m2 = 1.0;
        a1 = 0.8;
        a2 = 0.5;
        ac1 = a1/2;
        ac2 = a2/2;
        gravity = 9.81;

        return true;
    }

    void PositionFeedforwardEffortController::commandCallback(const std_msgs::Float64MultiArrayConstPtr &msg){
        
        ROS_WARN("Received new command");
        for (int i = 0; i < num_joints; i++){
            joint_reference_position[i] = msg->data[i];
            ROS_WARN("Reference Joint %d value %f", i, msg->data[i]);
        }
        
    }

    void PositionFeedforwardEffortController::starting(const ros::Time& time) { 
        
        for (int i = 0; i < num_joints; i++){
            pid_controller[i].reset();
        }
    
    }

    void PositionFeedforwardEffortController::update(const ros::Time& time, const ros::Duration& period){

        for (int i = 0; i < num_joints; i++){
            // Get Joint State
            joint_position[i] = joints_hw[i].getPosition();
            joint_velocity[i] = joints_hw[i].getVelocity();

            // Compute Error
            error[i] =  joint_reference_position[i] - joint_position[i];
            // error_dot[i] = joint_reference_velocity[i] - joint_velocity[i];
            error_dot[i] = (error[i] - error_old[i]) / period.toSec();
        }

        // Update Model
        updateDynamics();

        // Compute Effort
        computeInverseDynamics();

        for (int i = 0; i < num_joints; i++){

            commanded_effort[i] = joint_feedforward_torques[i] + pid_controller[i].computeCommand(error[i], error_dot[i], period);
            
            //ROS_INFO("Joint #%d Command %f", i, commanded_effort[i]);
            joints_hw[i].setCommand(commanded_effort[i]);
            
            // Store old error
            error_old[i] = error[i];
        }
    }

    void PositionFeedforwardEffortController::updateDynamics(){

        M[0][0] = m1*ac1*ac1 + m2*(a1*a1 + ac2*ac2 + 2*a1*ac2*cos(joint_position[1]) );
        M[0][1] = m2*(ac2*ac2 +  a1*ac2*cos(joint_position[1]) );
        M[1][0] = M[0][1];
        M[1][1] = m2*ac2*ac2;

        V[0] = -m2*a1*ac2*sin(joint_position[1])*( 2*joint_velocity[0]*joint_velocity[1] + joint_velocity[1]*joint_velocity[1]);
        V[0] += (m1*ac1 + m2*a1)*gravity*cos(joint_position[0]) + m2*gravity*ac2*cos( joint_position[0] + joint_position[1]);
        
        V[1] = -m2*a1*ac2*sin(joint_position[1])*( joint_velocity[0]*joint_velocity[0]);
        V[1] += m2*gravity*ac2*cos( joint_position[0] + joint_position[1]);

    }

    void PositionFeedforwardEffortController::computeInverseDynamics(){

        for (int i = 0; i < num_joints; i++){
            joint_feedforward_torques[i] = M[i][0]*joint_reference_acc[0] + M[i][1]*joint_reference_acc[1] + V[i]; 
        }

    }

}