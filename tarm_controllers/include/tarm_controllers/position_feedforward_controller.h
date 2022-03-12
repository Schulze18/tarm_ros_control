#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <control_toolbox/pid.h>
#include <std_msgs/Float64MultiArray.h>

namespace tarm_controllers{

    class PositionFeedforwardEffortController : public controller_interface::Controller<hardware_interface::EffortJointInterface>{

        public:
            PositionFeedforwardEffortController();

            bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &nh);
        
            void update(const ros::Time& time, const ros::Duration& period);

            void starting(const ros::Time& time);

            void updateDynamics();

            void computeInverseDynamics();

            void commandCallback(const std_msgs::Float64MultiArrayConstPtr &msg);
        
        private:
            std::vector<hardware_interface::JointHandle> joints_hw;
            std::vector<control_toolbox::Pid> pid_controller;
            ros::Subscriber command_subscriber;
            // Joints Vector
            static const int num_joints = 2; 
            std::vector<double> joint_feedforward_torques;
            std::vector<double> commanded_effort;
            std::vector<double> joint_position;
            std::vector<double> joint_velocity;
            std::vector<double> joint_acc;
            std::vector<double> joint_reference_position;
            std::vector<double> joint_reference_velocity;
            std::vector<double> joint_reference_acc;
            std::vector<double> error;
            std::vector<double> error_old;
            std::vector<double> error_dot;
            std::vector<double> joint_delt_cmd;
            // Tool Data
            double tool_position[2];
            double tool_velocity[2];
            double tool_reference_position[2];
            double tool_jacobian[2][2];
            double constant[2];
            // EoM Matrices
            double M[num_joints][num_joints];
            double V[num_joints];
            // Robot Parameters
            double m1, m2;
            double ac1, ac2;
            double a1, a2;
            double gravity;

    };
    PLUGINLIB_EXPORT_CLASS(tarm_controllers::PositionFeedforwardEffortController, controller_interface::ControllerBase);
}//namespace