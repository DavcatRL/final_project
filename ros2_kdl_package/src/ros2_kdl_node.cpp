#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;
using namespace std::placeholders;

class Iiwa_pub_sub : public rclcpp::Node
{
    public:
        Iiwa_pub_sub()
        : Node("ros2_kdl_node"), 
        node_handle_(std::shared_ptr<Iiwa_pub_sub>(this))
        {
            // declare external parameters cmd_interface and ctrl
            declare_parameter("cmd_interface", "position");     // default to "position"
            declare_parameter("ctrl", "velocity");              // default value to "velocity"
            get_parameter("cmd_interface", cmd_interface_);
            get_parameter("ctrl", ctrl_);

            // declare traj_type parameter (linear, circular)
            declare_parameter("traj_type", "quintic");
            get_parameter("traj_type", traj_type_);

            // declare s_type parameter (trapezoidal, cubic)
            declare_parameter("s_type", "trapezoidal");
            get_parameter("s_type", s_type_);

            // prints
            RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());
            if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "effort" ))
            {
                RCLCPP_ERROR(get_logger(),"Selected cmd interface is not valid! Use 'position', 'velocity' or 'effort' instead...");
                return;
            }

            RCLCPP_INFO(get_logger(),"Current ctrl is: '%s'", ctrl_.c_str());
            if (!(ctrl_ == "velocity" || ctrl_ == "velocity_null"))
            {
                RCLCPP_ERROR(get_logger(),"Selected cmd interface is not valid! Use 'velocity', 'velocity_null' or 'vision_ctrl' instead...");
                return;
            }

            RCLCPP_INFO(get_logger(),"Current trajectory type is: '%s'", traj_type_.c_str());
            if (!(traj_type_ == "linear" || traj_type_ == "circular" || traj_type_ == "quintic"))
            {
                RCLCPP_INFO(get_logger(),"Selected traj type is not valid!"); return;
            }


            RCLCPP_INFO(get_logger(),"Current s type is: '%s'", s_type_.c_str());
            if (!(s_type_ == "trapezoidal" || s_type_ == "cubic"))
            {
                RCLCPP_INFO(get_logger(),"Selected s type is not valid!"); return;
            }

            // internal parameters initialization
            iteration_ = 0; 
            t_ = 0;
            joint_state_available_ = false; 
            quintic_computed = false;                       
            double traj_duration = 1.5, acc_duration = 0.5, traj_radius = 0.15; // Plan trajectory

            // ROBOT INITIALIZATION
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "iiwa/robot_state_publisher");
            while (!parameters_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }
            auto parameter = parameters_client->get_parameters({"robot_description"});

            // create KDLrobot structure
            KDL::Tree robot_tree;
            if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
                std::cout << "Failed to retrieve robot_description param!";
            }
            robot_ = std::make_shared<KDLRobot>(robot_tree); 
            unsigned int nj = robot_->getNrJnts(); 
            
            // Create joint array
            KDL::JntArray q_min(nj), q_max(nj);
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96;    //-2*M_PI,-2*M_PI;
            q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96;          //2*M_PI, 2*M_PI;           
            robot_->setJntLimits(q_min,q_max);            
            joint_positions_.resize(nj); 
            joint_velocities_.resize(nj); 
            joint_positions_cmd_.resize(nj); 
            joint_velocities_cmd_.resize(nj); 
            joint_efforts_cmd_.resize(nj); 
            joint_efforts_cmd_.data.setZero();

            // Subscriber to jnt states
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/iiwa/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

            // Wait for the joint_state topic
            while(!joint_state_available_){
                RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
                rclcpp::spin_some(node_handle_);
            }

            // Update KDLrobot object
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            KDL::Frame f_T_ee = KDL::Frame::Identity();
            robot_->addEE(f_T_ee);
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

            // Compute EE frame
            init_cart_pose_ = robot_->getEEFrame();
            Eigen::Vector3d i_rotation;            
            init_cart_pose_.M.GetRPY(i_rotation(0), i_rotation(1), i_rotation(2));  
            RCLCPP_INFO(this->get_logger(), "Initial rotation: roll=%.3f, pitch=%.3f, yaw=%.3f",
                i_rotation(0), i_rotation(1), i_rotation(2));

            // Compute IK
            KDL::JntArray q(nj);
            robot_->getInverseKinematics(init_cart_pose_, q);

            // Initialize controller
            controller_ = std::make_shared<KDLController>(*robot_);

            // EE's trajectory initial position
            Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data));

            // desired end coordinates
            this->declare_parameter("end_position.x", init_position[0]);
            this->declare_parameter("end_position.y", -init_position[1]);
            this->declare_parameter("end_position.z", (init_position[2] - 0.1));
            this->declare_parameter("end_position.yaw", (i_rotation(2) - 0.78));

            this->get_parameter("end_position.x", end_x_);
            this->get_parameter("end_position.y", end_y_);
            this->get_parameter("end_position.z", end_z_);
            this->get_parameter("end_position.yaw", final_yaw);

            end_position_ << end_x_, end_y_, end_z_;

            RCLCPP_INFO(this->get_logger(), "Initial pose (KDL): x=%.3f, y=%.3f, z=%.3f",
                init_cart_pose_.p.x(), init_cart_pose_.p.y(), init_cart_pose_.p.z());

            RCLCPP_INFO(this->get_logger(), "Computed init_position (target base): x=%.3f, y=%.3f, z=%.3f",
                init_position[0], init_position[1], init_position[2]);

            RCLCPP_INFO(this->get_logger(), "Computed end_position (target base): x=%.3f, y=%.3f, z=%.3f, yaw=%.3f",
                end_position_[0], end_position_[1], end_position_[2]);


            // TRAJECTORY PLANNING
            if(traj_type_ == "linear"){
                planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position_);
                if(s_type_ == "trapezoidal")
                {
                    p_ = planner_.linear_traj_trapezoidal(t_);
                }else if(s_type_ == "cubic")
                {
                    p_ = planner_.linear_traj_cubic(t_);
                }
            } 
            else if(traj_type_ == "circular")
            {
                planner_ = KDLPlanner(traj_duration, init_position, traj_radius, acc_duration);
                if(s_type_ == "trapezoidal")
                {
                    p_ = planner_.circular_traj_trapezoidal(t_);
                }else if(s_type_ == "cubic")
                {
                    p_ = planner_.circular_traj_cubic(t_);
                }
            }
            else if(traj_type_ == "quintic")
            {
                Eigen::Vector3d delta = end_position_ - init_position;
                double delta_yaw = final_yaw - i_rotation(2);
                error << delta.x(), delta.y(), delta.z(), delta_yaw;
                RCLCPP_INFO(this->get_logger(), "error: x=%.3f, y=%.3f, z=%.3f, yaw=%.3f",
                            error[0], error[1], error[2], error[3]);
                quintic_coeffs = KDLPlanner::compute_quintic(traj_duration, error);
                quintic_computed = true;
            }
            
            if(cmd_interface_ == "position"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa/iiwa_arm_controller/commands", 10);

                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Send joint position commands
                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_(i);
                }
            }
            else if(cmd_interface_ == "velocity"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa/velocity_controller/commands", 10);

                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                        std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Set joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i);
                }
            }
            else if(cmd_interface_ == "effort"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa/effort_controller/commands", 10);

                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Set joint effort commands
                for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                    desired_commands_[i] = joint_efforts_cmd_(i);
                }
            } 

            // Create msg and publish
            /*
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);
            */

            RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
        }

    private:

        // COMMAND PUBLISHER
        void cmd_publisher(){
            
            // internal variables definition and update
            iteration_ = iteration_ + 1;

            // define trajectory UNA VOLTA
            // la durata è fasulla, qui il dt è 0.01 ma in realtà dato che il publisher pubblica ogni 0.1, tutto scala di un ordine di grandezza
            // total time invece di essere 1.5, è 15
            double total_time = 1.5; 
            int trajectory_len = 150; 
            int loop_rate = trajectory_len / total_time;
            double dt = 1.0 / loop_rate;
            Kp_ = 5;
            t_+=dt;

            // EE 
            KDL::Frame cartpos;         // current EE frame
            Eigen::Vector3d orient;     // current orientation
            Eigen::Vector4d EEpose;     // current pose : x,y,z + yaw

            // trajectory planners != quintic
            KDL::Frame desFrame;        // setpoint
            Eigen::Vector3d error_p;    // error in position
            Eigen::Vector3d o_error;    // error in orientation

            // trajectory planning = quintic
            KDL::Frame setpoint;        // setpoint

            // Compute EE frame and pose
            cartpos = robot_->getEEFrame(); 
            cartpos.M.GetRPY(orient(0), orient(1), orient(2)); 
            EEpose << toEigen(cartpos.p), orient(2);

            RCLCPP_INFO(this->get_logger(), "ee_pose: x=%.3f, y=%.3f, z=%.3f, yaw=%.3f",
                EEpose[0], EEpose[1], EEpose[2], EEpose[3]);
           
            // planners : compute setpoints
            if (t_ < total_time){

                // Retrieve the trajectory point based on the trajectory type
                if(traj_type_ == "linear"){
                    if(s_type_ == "trapezoidal")
                    {
                        p_ = planner_.linear_traj_trapezoidal(t_);
                    }
                    else if(s_type_ == "cubic")
                    {
                        p_ = planner_.linear_traj_cubic(t_);
                    }
                } 
                else if(traj_type_ == "circular")
                {
                    if(s_type_ == "trapezoidal")
                    {
                        p_ = planner_.circular_traj_trapezoidal(t_);
                    }
                    else if(s_type_ == "cubic")
                    {
                        p_ = planner_.circular_traj_cubic(t_);
                    }
                }
                else if(traj_type_ == "quintic")
                {
                    setpoint = KDLPlanner::compute_trajectory_setpoint(t_, error, quintic_coeffs, init_cart_pose_); 
                }
                
                if(traj_type_!="quintic"){
                    // Compute desired Frame: same orientation, p position
                    desFrame.M = cartpos.M; 
                    desFrame.p = toKDL(p_.pos); 

                    // compute errors
                    error_p = computeLinearError(p_.pos, Eigen::Vector3d(cartpos.p.data));
                    o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));
                    std::cout << "The error norm is : " << error_p.norm() << std::endl;
                }

                // control action computing
                if(cmd_interface_ == "position"){
                    KDL::Frame nextFrame; nextFrame.M = cartpos.M; nextFrame.p = cartpos.p + (toKDL(p_.vel) + toKDL(Kp_*error_p))*dt; 

                    // Compute IK
                    joint_positions_cmd_ = joint_positions_;
                    robot_->getInverseKinematics(nextFrame, joint_positions_cmd_);
                }
                else if(cmd_interface_ == "velocity"){
                    if(ctrl_ == "velocity"){
                        // Compute differential IK
                        Eigen::VectorXd cartvel; cartvel << p_.vel + Kp_*error_p, o_error;
                        joint_velocities_cmd_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;                        
                    }
                    else if(ctrl_ == "velocity_null"){
                        RCLCPP_INFO(this->get_logger(), "sono entrato nel controllore");
                        joint_velocities_cmd_.data = controller_->velocity_ctrl_null(setpoint,Kp_);
                    }
                }
                else if(cmd_interface_ == "effort"){
                    joint_efforts_cmd_.data[0] = 0.1*std::sin(2*M_PI*t_/total_time);
                }

                // Update KDLrobot structure
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                if(cmd_interface_ == "position"){
                    // Set joint position commands
                    for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                        desired_commands_[i] = joint_positions_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "velocity"){
                    // Set joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = joint_velocities_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "effort"){
                    // Set joint effort commands
                    for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                        desired_commands_[i] = joint_efforts_cmd_(i);
                    }
                } 

                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);

            }
            else{
                RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");
                
                // Send joint velocity commands
                if(cmd_interface_ == "position"){
                    // Set joint position commands
                    for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                        desired_commands_[i] = joint_positions_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "velocity"){
                    // Set joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = 0.0;
                    }
                }
                else if(cmd_interface_ == "effort"){
                    // Set joint effort commands
                    for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                        desired_commands_[i] = joint_efforts_cmd_(i);
                    }
                }
                
                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
            }
        }

        // JOINT SUBSCRIBER
        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){
            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
            }
        }

        // VARIABLES DECLARATION

        // subscribers/publishers
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::TimerBase::SharedPtr subTimer_;
        rclcpp::Node::SharedPtr node_handle_;

        // robot configuration
        bool joint_state_available_;
        std::shared_ptr<KDLRobot> robot_;
        KDL::JntArray joint_positions_;
        KDL::JntArray joint_velocities_;

        // robot control
        std::shared_ptr<KDLController> controller_;
        std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        KDL::JntArray joint_positions_cmd_;
        KDL::JntArray joint_velocities_cmd_;
        KDL::JntArray joint_efforts_cmd_;

        // trajectory : settings and trasformations
        KDL::Frame init_cart_pose_;
        Eigen::Vector3d end_position_;
        Eigen::Vector4d error;
        double end_x_, end_y_, end_z_, final_yaw;
        double Kp_;
        double t_;
        int iteration_;

        // trajectory : planning
        KDLPlanner planner_;
        trajectory_point p_;
        Eigen::Matrix<double, 6, 1> quintic_coeffs;
        bool quintic_computed;

        // external parameters
        std::string cmd_interface_;
        std::string ctrl_;
        std::string traj_type_;
        std::string s_type_;
};

 
int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 1;
}