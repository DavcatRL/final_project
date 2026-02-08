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

#include <functional>
#include <thread>

#include "ros2_kdl_package/action/ros2_kdl_traj.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;
using namespace std::placeholders;
using Ros2KdlTraj = ros2_kdl_package::action::Ros2KdlTraj;
using GoalHandleRos2KdlTraj = rclcpp_action::ServerGoalHandle<Ros2KdlTraj>;

class Iiwa_action : public rclcpp::Node
{
    public:
        Iiwa_action()
        : Node("ros2_kdl_action"), 
        node_handle_(std::shared_ptr<Iiwa_action>(this))
        {
            // internal parameters initialization
            iteration_ = 0; 
            t_ = 0;
            joint_state_available_  = false; 
            quintic_computed        = false;
            executing_              = false;                       

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

            std::cout << "=== FULL TREE SEGMENTS ===" << std::endl;
            unsigned int count = 0;
            for(auto it = robot_tree.getSegments().begin(); it != robot_tree.getSegments().end(); ++it) {
                std::cout << count++ << ": " << it->first;
                
                KDL::Segment seg = it->second.segment;
                KDL::Joint jnt = seg.getJoint();
                
                if(jnt.getType() != KDL::Joint::None) {
                    std::cout << " [JOINT: " << jnt.getName() 
                            << " type: " << jnt.getTypeName() << "]";
                } else {
                    std::cout << " [FIXED]";
                }
                
                // Massa del segmento
                KDL::RigidBodyInertia inertia = seg.getInertia();
                if(inertia.getMass() > 0) {
                    std::cout << " mass: " << inertia.getMass() << "kg";
                }
                
                std::cout << std::endl;
            }
            std::cout << "=== END TREE ===" << std::endl;

            robot_ = std::make_shared<KDLRobot>(robot_tree); 
            robot_->addFakeGripper();
            robot_->updateChainSolvers();
            unsigned int nj = robot_->getNrJnts();
            
            // Create joint array
            KDL::JntArray q_min(nj), q_max(nj);
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96;    //-2*M_PI,-2*M_PI;
            q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96;          //2*M_PI, 2*M_PI;           
            robot_->setJntLimits(q_min,q_max);    
            joint_positions_.resize(nj);         
            joint_velocities_.resize(nj); 
            joint_velocities_cmd_.resize(nj);
            RCLCPP_INFO(this->get_logger(), "number of joints: %u", nj); 
            std::cout << "JntArray size: " << joint_positions_.rows() << std::endl;

            // Subscriber to jnt states
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/iiwa/joint_states", 10, std::bind(&Iiwa_action::joint_state_subscriber, this, std::placeholders::_1));

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

            // Compute IK
            KDL::JntArray q(nj);
            robot_->getInverseKinematics(init_cart_pose_, q);

            // EE's trajectory initial position 
            i_spatial_coordinates << Eigen::Vector3d(init_cart_pose_.p.data);            
            init_cart_pose_.M.GetRPY(i_rotation(0), i_rotation(1), i_rotation(2));

            RCLCPP_INFO(this->get_logger(), "Initial pose (KDL): x=%.3f, y=%.3f, z=%.3f",
                init_cart_pose_.p.x(), init_cart_pose_.p.y(), init_cart_pose_.p.z());

            RCLCPP_INFO(this->get_logger(), "Computed i_spatial_coordinates (target base): x=%.3f, y=%.3f, z=%.3f",
                i_spatial_coordinates[0], i_spatial_coordinates[1], i_spatial_coordinates[2]);

            RCLCPP_INFO(this->get_logger(), "Initial rotation: roll=%.3f, pitch=%.3f, yaw=%.3f",
                i_rotation(0), i_rotation(1), i_rotation(2));

            // Initialize controller and publisher
            controller_ = std::make_shared<KDLController>(*robot_);
            cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa/velocity_controller/commands", 10);
            
            // ACTION SERVER INITIALIZATION
            this->action_server_ = rclcpp_action::create_server<Ros2KdlTraj>(
                this,
                "ros2_kdl_traj",
                std::bind(&Iiwa_action::handle_goal, this, _1, _2),
                std::bind(&Iiwa_action::handle_cancel, this, _1),
                std::bind(&Iiwa_action::handle_accepted, this, _1)
            );
        }

    private:

        // JOINT SUBSCRIBER
        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){
            joint_state_available_ = true;
            for (unsigned int i  = 0; i < 7; i++){ //i < sensor_msg.position.size()
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
            }
        }

        // ACTION SERVER DEFINITION
        
        // accettazione del goal
        rclcpp_action::GoalResponse handle_goal(
                                const rclcpp_action::GoalUUID & uuid,           // identificativo del goal : UUID = Universally Unique Identifier
                                std::shared_ptr<const Ros2KdlTraj::Goal> goal)  // goal message da leggere
        {
            (void)uuid; // non fa nulla, zittisce il compilatore se viene passato un rgomento non utilizzato (warning)

            // controllo che il controllore sia libero

            if (executing_) {
                RCLCPP_WARN(this->get_logger(), "Rejected goal: controller busy");
                return rclcpp_action::GoalResponse::REJECT;
            }

            // controlli sulla validità della traiettoria

            if (goal->traj_duration <= 0.0){
                RCLCPP_WARN(this->get_logger(), "Rejected goal: traj_duration <= 0");
                return rclcpp_action::GoalResponse::REJECT;
            }

            if (!std::isfinite(goal->x) ||
                !std::isfinite(goal->y) ||
                !std::isfinite(goal->z) ||
                !std::isfinite(goal->yaw)){
                    RCLCPP_WARN(this->get_logger(), "Rejected goal: infinite coordinates");
                    return rclcpp_action::GoalResponse::REJECT;
            }  

            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }


        // lancio del nuovo thread
        void handle_accepted(const std::shared_ptr<GoalHandleRos2KdlTraj> goal_handle)
        {
            // this needs to return quickly to avoid blocking the executor, so spin up a new thread
            std::thread{std::bind(&Iiwa_action::execute, this, _1), goal_handle}.detach();
        }

        
        // cancellazione 
        rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleRos2KdlTraj> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        // executing
        void execute(const std::shared_ptr<GoalHandleRos2KdlTraj> goal_handle)
        {
            // starting execution
            RCLCPP_INFO(this->get_logger(), "Executing action goal");
            executing_ = true;
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<Ros2KdlTraj::Feedback>();
            auto result = std::make_shared<Ros2KdlTraj::Result>();

            // EE 
            KDL::Frame cartpos;         // current EE frame
            KDL::Frame final_pose;
            Eigen::Vector3d orient;     // current orientation
            Eigen::Vector4d EEpose;     // current pose : x,y,z + yaw

            // define trajectory and parameters
            KDL::Frame setpoint;

            t_ = 0.0;
            total_time  = goal->traj_duration;
            double dt_traj = total_time / trajectory_len;
            double dt_min  = 0.001;
            dt = std::max(dt_traj, dt_min);

            rclcpp::Rate rate(1.0 / dt);  // sleep rate in Hz

            end_x_ = goal->x;
            end_y_ = goal->y;
            end_z_ = goal->z;
            final_yaw = goal->yaw;
            end_position_ << end_x_, end_y_, end_z_;

            // TRAJECTORY PLANNING: quintic polynomial
            Eigen::Vector3d delta   = end_position_ - i_spatial_coordinates;
            double delta_yaw        = final_yaw - i_rotation(2);             

            error << delta.x(), delta.y(), delta.z(), delta_yaw;
            RCLCPP_INFO(this->get_logger(), "error: x=%.3f, y=%.3f, z=%.3f, yaw=%.3f",
                        error[0], error[1], error[2], error[3]);
            
            if(!quintic_computed){
                quintic_coeffs = KDLPlanner::compute_quintic(total_time, error);
                quintic_computed = true;
            }

            RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");

            for (int k = 0; k < trajectory_len && rclcpp::ok(); ++k){

                // Compute current EE frame and pose
                cartpos = robot_->getEEFrame(); 
                cartpos.M.GetRPY(orient(0), orient(1), orient(2)); 
                EEpose << toEigen(cartpos.p), orient(2);

                // handling the cancelation
                if (goal_handle->is_canceling()) {
                    RCLCPP_WARN(this->get_logger(), "Goal canceled");

                    executing_ = false;
                    quintic_computed = false;

                    result->x_f     = EEpose(0);
                    result->y_f     = EEpose(1);
                    result->z_f     = EEpose(2);
                    result->yaw_f   = EEpose(3);

                    // stopping the robot
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = 0.0;
                    }
                    std_msgs::msg::Float64MultiArray cmd_msg;
                    cmd_msg.data = desired_commands_;
                    cmdPublisher_->publish(cmd_msg);

                    // updating the initial position as the current position
                    i_spatial_coordinates << EEpose(0), EEpose(1), EEpose(2);
                    i_rotation(2) = EEpose(3);

                    goal_handle->canceled(result);

                    return;
                }

                // computing next setpoint
                t_+=dt;
                // RCLCPP_INFO(this->get_logger(), "current t_: %.3f", t_);
                setpoint = KDLPlanner::compute_trajectory_setpoint(t_, error, quintic_coeffs, init_cart_pose_); // qui errore, il frame init cart pose, non è aggiornato

                // send feedback
                feedback->current_x     = EEpose(0);
                feedback->current_y     = EEpose(1);
                feedback->current_z     = EEpose(2);
                feedback->current_yaw   = EEpose(3);
                feedback->advancement   = (t_ / total_time) * 100;
                goal_handle->publish_feedback(feedback);
                
                // computing the velocity commands
                joint_velocities_cmd_.data = controller_->velocity_ctrl_null(setpoint,Kp_);

                // Update KDLrobot structure
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_cmd_(i);
                }

                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);

                rate.sleep();   // compensa il tempo già trascorso: problema?
            }

            // last robot update
            robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

            final_pose = robot_->getEEFrame();
            final_pose.M.GetRPY(orient(0), orient(1), orient(2)); 
            EEpose << toEigen(final_pose.p), orient(2);

            result->x_f     = EEpose(0);
            result->y_f     = EEpose(1);
            result->z_f     = EEpose(2);
            result->yaw_f   = EEpose(3);

            // stopping the robot
            for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                desired_commands_[i] = 0.0;
            }
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);

            // updating the initial position as the current position
            i_spatial_coordinates << EEpose(0), EEpose(1), EEpose(2);
            i_rotation(2) = EEpose(3);
            

            if (rclcpp::ok()) {
                goal_handle->succeed(result);

                // resetting parameters
                executing_ = false;
                quintic_computed = false;
                init_cart_pose_ = final_pose;

                RCLCPP_INFO(this->get_logger(), "Goal succeeded. Final pose: x=%.3f y=%.3f z=%.3f yaw=%.3f",
                                            result->x_f, result->y_f, result->z_f, result->yaw_f);
                return;
            }
        } 


        // VARIABLES DECLARATION

        // subscribers/publishers
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        rclcpp::Node::SharedPtr node_handle_;
        rclcpp_action::Server<Ros2KdlTraj>::SharedPtr action_server_;   // AGGIORNA IL NOME DEL MESSAGGIO

        // robot configuration
        bool joint_state_available_;
        std::shared_ptr<KDLRobot> robot_;
        KDL::JntArray joint_velocities_;
        KDL::JntArray joint_positions_;

        KDL::Frame init_cart_pose_;
        Eigen::Vector3d i_spatial_coordinates;
        Eigen::Vector3d i_rotation;

        // robot control
        std::shared_ptr<KDLController> controller_;
        std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        KDL::JntArray joint_velocities_cmd_;

        // trajectory : settings and trasformations
        Eigen::Vector3d end_position_;
        Eigen::Vector4d error;
        double end_x_, end_y_, end_z_, final_yaw;
        double Kp_ = 5;
        double t_;
        int iteration_;
        double total_time;
        int trajectory_len = 150;
        double dt;

        // trajectory : planning
        Eigen::Matrix<double, 6, 1> quintic_coeffs;
        bool quintic_computed;

        // action
        bool executing_;
};

 
int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_action>());
    rclcpp::shutdown();
    return 1;
}