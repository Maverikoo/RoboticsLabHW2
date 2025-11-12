#ifndef KDLControl
#define KDLControl

#include "Eigen/Dense"
#include "kdl_robot.h"
#include "utils.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
class KDLController
{

public:

    KDLController(KDLRobot &_robot);

    Eigen::VectorXd idCntr(KDL::JntArray &_qd,
                           KDL::JntArray &_dqd,
                           KDL::JntArray &_ddqd,
                           double _Kp,
                           double _Kd);

    Eigen::VectorXd idCntr(KDL::Frame &_desPos,
                           KDL::Twist &_desVel,
                           KDL::Twist &_desAcc,
                           double _Kpp,
                           double _Kpo,
                           double _Kdp,
                           double _Kdo);
                           
  
    KDL::JntArray velocity_ctrl_null(const Eigen::Vector3d& p_des,
                                 double Kp,
                                 double lambda,
                                 const Eigen::VectorXd& q_min,
                                 const Eigen::VectorXd& q_max);

    KDL::JntArray vision_ctrl(const geometry_msgs::msg::PoseStamped& aruco_pose,
                              double Kp,
                              double lambda,
                              const Eigen::VectorXd& q_min,
                              const Eigen::VectorXd& q_max);

private:

    KDLRobot* robot_;
};

#endif