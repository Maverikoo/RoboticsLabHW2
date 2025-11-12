// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
 
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>

// Headers ROS Standard
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" // (Include corretto)


// Headers KDL
#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"

// ***** MODIFICHE PER ACTION SERVER COMPONENT *****
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "ros2_kdl_package/action/execute_trajectory.hpp" 
// ***** FINE MODIFICHE *****
 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

// Spostiamo gli using comuni qui
using TrajectoryAction = ros2_kdl_package::action::ExecuteTrajectory;
using GoalHandle = rclcpp_action::ServerGoalHandle<TrajectoryAction>;


// =================================================================================
// ***** CLASSE ORIGINALE TRASFORMATA IN COMPONENTE ACTION SERVER *****
//
class Iiwa_pub_sub : public rclcpp::Node
{
    public:
        explicit Iiwa_pub_sub(const rclcpp::NodeOptions & options)
        // ***** BUG 2 IGNORATO (Come da tua richiesta) *****
        : Node("ros2_kdl_node_1c", options) 
        {
            RCLCPP_INFO(this->get_logger(), "Avvio del KdlNode1c (Iiwa_pub_sub) Action Server...");

            // --- Inizializzazione Parametri ---
            this->declare_parameter("cmd_interface", "velocity");
            this->get_parameter("cmd_interface", cmd_interface_);
            this->declare_parameter("traj_type", "linear");
            this->get_parameter("traj_type", traj_type_);
            this->declare_parameter("s_type", "trapezoidal");
            this->get_parameter("s_type", s_type_);
            this->declare_parameter("ctrl", "velocity_ctrl");
            this->get_parameter("ctrl", ctrl_);
            this->declare_parameter("traj_duration", 1.5);
            this->get_parameter("traj_duration", traj_duration_);
            this->declare_parameter("acc_duration", 0.5);
            this->get_parameter("acc_duration", acc_duration_);
            this->declare_parameter("total_time", 1.5);
            this->get_parameter("total_time", total_time_);
            this->declare_parameter("trajectory_len", 150);
            this->get_parameter("trajectory_len", trajectory_len_);
            this->declare_parameter("Kp", 5.0);
            this->get_parameter("Kp", Kp_);
            this->declare_parameter("lambda", 0.5);
            this->get_parameter("lambda", lambda_);

            iteration_ = 0; t_ = 0;
            joint_state_available_ = false; 
            aruco_pose_available_=false;

            // --- Setup KDL Robot ---
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "robot_state_publisher");
            while (!parameters_client->wait_for_service(std::chrono::seconds(1))) { // Corretto (senza _literals)
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrotto... Exiting.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Servizio robot_state_publisher non disponibile, attendo...");
            }
            auto parameter = parameters_client->get_parameters({"robot_description"});

            KDL::Tree robot_tree;
            if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
                RCLCPP_ERROR(this->get_logger(), "Impossibile recuperare robot_description!");
                return;
            }
            robot_ = std::make_shared<KDLRobot>(robot_tree);  
            
            unsigned int nj = robot_->getNrJnts();
            KDL::JntArray q_min(nj), q_max(nj);
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96;
            q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96;
           
            q_min_ = q_min.data;
            q_max_ = q_max.data;
           
            robot_->setJntLimits(q_min,q_max);            
            joint_positions_.resize(nj); 
            joint_velocities_.resize(nj); 
            joint_positions_cmd_.resize(nj); 
            joint_velocities_cmd_.resize(nj); 
            joint_efforts_cmd_.resize(nj); joint_efforts_cmd_.data.setZero();

            // --- Subscriber ---
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

            // Ascolta il topic REALE di ArUco
            aruco_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/aruco_single/pose", 10, 
                std::bind(&Iiwa_pub_sub::aruco_pose_callback, this, std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(), "Attendo il primo messaggio /joint_states...");
            while(!joint_state_available_){
                rclcpp::spin_some(this->get_node_base_interface());
            }
            RCLCPP_INFO(this->get_logger(), "Joint states ricevuti.");

            // Update KDLrobot (Corretto)
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            KDL::Frame T_ee_camera;
            T_ee_camera.M = KDL::Rotation::RPY(-1.5708, 0.0, -1.5708);
            T_ee_camera.p = KDL::Vector(0.0, 0.0, 0.0); 
            robot_->addEE(T_ee_camera);
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            init_cart_pose_ = robot_->getEEFrame();

            // --- Inizializza Controller ---
            controller_ = std::make_shared<KDLController>(*robot_);

            // --- Publisher ---
            if(cmd_interface_ == "position"){
                cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
            }
            else if(cmd_interface_ == "velocity"){
                cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
            }
            else if(cmd_interface_ == "effort"){
                cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
            }

            // --- Creazione Action Server ---
            int loop_rate = trajectory_len_ / total_time_;
            // Timer Corretto
            double dt_in_secondi = 1.0 / loop_rate;
            double dt_in_millisecondi = dt_in_secondi * 1000.0; 
            
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(dt_in_millisecondi)), 
                std::bind(&Iiwa_pub_sub::cmd_publisher_callback, this));
            timer_->cancel();

            this->action_server_ = rclcpp_action::create_server<TrajectoryAction>(
                this,
                "execute_trajectory",
                std::bind(&Iiwa_pub_sub::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&Iiwa_pub_sub::handle_cancel, this, std::placeholders::_1),
                std::bind(&Iiwa_pub_sub::handle_accepted, this, std::placeholders::_1)
            );

            RCLCPP_INFO(this->get_logger(), "Iiwa_pub_sub Action Server pronto e in attesa di un goal.");
        }

    // Distruttore
    virtual ~Iiwa_pub_sub() {}

    private:
        // --- Callback per l'Action Server ---
        
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const TrajectoryAction::Goal> goal)
        {
            (void)uuid;
            RCLCPP_INFO(this->get_logger(), "Ricevuto goal: target=[%.2f, %.2f, %.2f]",
                goal->target_end_position.x,
                goal->target_end_position.y,
                goal->target_end_position.z);

            if (!timer_->is_canceled()) {
                RCLCPP_WARN(this->get_logger(), "Rifiuto goal! Una traiettoria è già attiva.");
                return rclcpp_action::GoalResponse::REJECT;
            }
            
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandle> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Ricevuta richiesta di annullamento goal...");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }


       void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
        {
            this->current_goal_handle_ = goal_handle;
            t_ = 0.0;
            iteration_ = 0;

            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            KDL::Frame start_frame = robot_->getEEFrame();
            Eigen::Vector3d init_position(start_frame.p.data);

            const auto& target = goal_handle->get_goal()->target_end_position;
            Eigen::Vector3d end_position; 
            end_position << target.x, target.y, target.z;
            
            this->final_target_position_ = end_position;
            init_cart_pose_ = start_frame; 

            // Se NON siamo in modalità visione, prepariamo il planner
            if(ctrl_ != "vision"){
                if(traj_type_ == "linear"){
                    planner_ = KDLPlanner(traj_duration_, acc_duration_, init_position, end_position);
                } 
                else if(traj_type_ == "circular")
                {
                    double traj_radius = (end_position - init_position).norm() / 2.0;
                    planner_ = KDLPlanner(traj_duration_, init_position, traj_radius, acc_duration_);
                }
            }    
            
            RCLCPP_INFO(this->get_logger(), "Goal Accettato. Avvio traiettoria...");
            timer_->reset();
        }


        // ***** CORREZIONE 1: LOGICA `cmd_publisher_callback` *****
        void cmd_publisher_callback()
        {
            if (current_goal_handle_->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "Traiettoria ANNULLATA.");
                timer_->cancel();
                send_zero_commands();
                auto result = std::make_shared<TrajectoryAction::Result>();
                result->success = false;
                current_goal_handle_->abort(result);
                return;
            }

            // --- Calcolo dei Comandi ---
            if (ctrl_ == "vision")
            {
                // ***** CORREZIONE 3: LOGICA TIMEOUT *****
                if (aruco_pose_available_ && (this->now().seconds() - this->latest_aruco_time_ > ARUCO_TIMEOUT_SEC))
                {
                    RCLCPP_WARN(this->get_logger(), "Timeout ArUco! Posa troppo vecchia. Fermo il robot.");
                    this->aruco_pose_available_ = false; 
                }
                // ***** FINE CORREZIONE *****

                if(cmd_interface_ == "velocity")
                {
                    if (!this->aruco_pose_available_)
                    {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Vision control active, ma non ho ancora ricevuto una posa ArUco.");
                        send_zero_commands();
                    }
                    else
                    {
                        joint_velocities_cmd_ = controller_->vision_ctrl(
                            this->latest_aruco_pose_, Kp_, lambda_, q_min_, q_max_
                        );
                        
                        Eigen::Vector3d P_o(latest_aruco_pose_.pose.position.x, latest_aruco_pose_.pose.position.y, latest_aruco_pose_.pose.position.z);
                        if (P_o.norm() > 1e-6) {
                            Eigen::Vector3d s = P_o / P_o.norm();
                            Eigen::Vector3d s_d(0.0, 0.0, 1.0);
                            double vision_error = (s_d - s).norm();

                            auto feedback = std::make_shared<TrajectoryAction::Feedback>();
                            feedback->current_error_norm = vision_error; 
                            current_goal_handle_->publish_feedback(feedback);
                        }
                    }
                }
                else
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Modalità 'vision' supportata solo con 'cmd_interface:=velocity'. Fermo il robot.");
                    send_zero_commands();
                }
            }
            else // (ctrl_ != "vision", quindi traiettoria)
            {
                int loop_rate = trajectory_len_ / total_time_;
                double dt = 1.0 / loop_rate;
                t_+=dt;
                iteration_++;

                if (t_ < total_time_) // La tua condizione
                { 
                    if(traj_type_ == "linear"){
                        if(s_type_ == "trapezoidal") { p_ = planner_.linear_traj_trapezoidal(t_); }
                        else if(s_type_ == "cubic") { p_ = planner_.linear_traj_cubic(t_); }
                    } 
                    else if(traj_type_ == "circular") {
                        if(s_type_ == "trapezoidal") { p_ = planner_.circular_traj_trapezoidal(t_); }
                        else if(s_type_ == "cubic") { p_ = planner_.circular_traj_cubic(t_); }
                    }

                    KDL::Frame cartpos = robot_->getEEFrame();
                    Eigen::Vector3d current_ee_pos(cartpos.p.data);
                    Eigen::Vector3d tracking_error = computeLinearError(p_.pos, current_ee_pos);
                    Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));
                    Eigen::Vector3d goal_error = computeLinearError(this->final_target_position_, current_ee_pos);
                    
                    auto feedback = std::make_shared<TrajectoryAction::Feedback>();
                    feedback->current_error_norm = goal_error.norm(); 
                    current_goal_handle_->publish_feedback(feedback);

                    const double success_threshold = 0.01;
                    if (goal_error.norm() < success_threshold)
                    {
                        RCLCPP_INFO(this->get_logger(), "Goal Raggiunto (Errore < %.4f m). Traiettoria COMPLETATA.", success_threshold);
                        timer_->cancel();
                        send_zero_commands();
                        auto result = std::make_shared<TrajectoryAction::Result>();
                        result->success = true;
                        current_goal_handle_->succeed(result);
                        return; // Usciamo dal callback
                    }

                    if(cmd_interface_ == "position"){
                        KDL::Frame nextFrame; nextFrame.M = cartpos.M; nextFrame.p = cartpos.p + (toKDL(p_.vel) + toKDL(Kp_*tracking_error))*dt; 
                        joint_positions_cmd_ = joint_positions_;
                        robot_->getInverseKinematics(nextFrame, joint_positions_cmd_);
                    }
                    else if(cmd_interface_ == "velocity"){
                        if (ctrl_ == "velocity_ctrl")
                        {
                            Vector6d cartvel; cartvel << p_.vel + Kp_*tracking_error, o_error;
                            joint_velocities_cmd_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                        }
                        else if (ctrl_ == "velocity_ctrl_null")
                        {
                            const Eigen::Vector3d p_des = p_.pos;
                            joint_velocities_cmd_ = controller_->velocity_ctrl_null(p_des, Kp_, lambda_, q_min_, q_max_);
                        }
                    }
                    else if(cmd_interface_ == "effort"){
                        joint_efforts_cmd_.data.setZero();
                        joint_efforts_cmd_.data[0] = 0.1*std::sin(2*M_PI*t_/total_time_);
                    }
                }
                else
                {
                    // --- Logica di Fine Traiettoria (Timeout) ---
                    timer_->cancel();
                    send_zero_commands();
                    KDL::Frame ee_final = robot_->getEEFrame();
                    Eigen::Vector3d final_error_vec = computeLinearError(this->final_target_position_, Eigen::Vector3d(ee_final.p.data));
                    double final_error = final_error_vec.norm();
                    auto result = std::make_shared<TrajectoryAction::Result>();

                    RCLCPP_WARN(this->get_logger(), "Traiettoria FALLITA (Timeout).");
                    RCLCPP_WARN(this->get_logger(),"Posizione EE Finale (attuale): [%.4f, %.4f, %.4f]", ee_final.p[0], ee_final.p[1], ee_final.p[2]);
                    RCLCPP_WARN(this->get_logger(), "Posizione EE Finale (desiderata): [%.4f, %.4f, %.4f]" ,this->final_target_position_[0], this->final_target_position_[1], this->final_target_position_[2]);
                    RCLCPP_WARN(this->get_logger(), "Errore finale: %.4f m", final_error);
                    
                    result->success = false; 
                    current_goal_handle_->abort(result);
                    return;
                }
            } 
            
            // ***** CORREZIONE 1: PUBLISH GLOBALE *****
            // Questo blocco ora è FUORI dalla logica if/else,
            // quindi funziona sia per "vision" che per gli altri.
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

            if(cmd_interface_ == "position"){
                for (long int i = 0; i < joint_positions_.data.size(); ++i) { desired_commands_[i] = joint_positions_cmd_(i); }
            }
            else if(cmd_interface_ == "velocity"){
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) { desired_commands_[i] = joint_velocities_cmd_(i); }
            }
            else if(cmd_interface_ == "effort"){
                for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) { desired_commands_[i] = joint_efforts_cmd_(i); }
            } 

            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);
        }
        
        // --- Altre Funzioni Membro ---

        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){
            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
            }
        }

        // ***** CORREZIONE 3: AGGIUNTA LOGICA TIMEOUT *****
        void aruco_pose_callback(const geometry_msgs::msg::PoseStamped& msg)
        {
            this->latest_aruco_pose_ = msg;
            this->aruco_pose_available_ = true;
            this->latest_aruco_time_ = this->now().seconds();
        }
        // ***** FINE CORREZIONE *****

        void send_zero_commands() {
            if(cmd_interface_ == "velocity"){
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) { desired_commands_[i] = 0.0; }
            }
            else if(cmd_interface_ == "position"){
                for (long int i = 0; i < joint_positions_.data.size(); ++i) { desired_commands_[i] = joint_positions_cmd_(i); }
            }
            else if(cmd_interface_ == "effort"){
                for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) { desired_commands_[i] = 0.0; }
            }
            
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);
        }

        

        // --- Variabili Membro ---

        // Action Server
        rclcpp_action::Server<TrajectoryAction>::SharedPtr action_server_;
        std::shared_ptr<GoalHandle> current_goal_handle_;
        
        // Publisher, Subscriber, Timer
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        rclcpp::TimerBase::SharedPtr timer_; 
        
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_subscriber_;

        // Comandi
        std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        KDL::JntArray joint_positions_;
        KDL::JntArray joint_velocities_;
        KDL::JntArray joint_positions_cmd_;
        KDL::JntArray joint_velocities_cmd_;
        KDL::JntArray joint_efforts_cmd_;

        // KDL
        std::shared_ptr<KDLRobot> robot_;
        std::shared_ptr<KDLController> controller_;
        KDLPlanner planner_;
        trajectory_point p_;

        // Stato
        int iteration_;
        bool joint_state_available_;
        double t_;
        KDL::Frame init_cart_pose_;
        Eigen::VectorXd q_min_;
        Eigen::VectorXd q_max_;
        Eigen::Vector3d final_target_position_;

        // Variabili per la visione
        bool aruco_pose_available_;
        geometry_msgs::msg::PoseStamped latest_aruco_pose_;
        
        // ***** CORREZIONE 3: DICHIARAZIONI TIMEOUT MANCANTI *****
        double latest_aruco_time_ = 0.0;
        const double ARUCO_TIMEOUT_SEC = 0.5; // Timeout di 0.5 secondi
        // ***** FINE CORREZIONE *****

        // Parametri
        std::string cmd_interface_;
        std::string traj_type_;
        std::string s_type_;
        std::string ctrl_;
        double traj_duration_;
        double acc_duration_;
        double total_time_;
        int    trajectory_len_;
        double Kp_;
        double lambda_;
};

// =================================================================================
// ***** MACRO DI REGISTRAZIONE COMPONENTE *****
//
RCLCPP_COMPONENTS_REGISTER_NODE(Iiwa_pub_sub)
// =================================================================================