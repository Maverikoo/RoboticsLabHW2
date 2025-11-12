#include "kdl_control.h"
#include "utils.h"

#include <algorithm>
#include <cmath>

KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q  = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e  = _qd.data  - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;

    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis()+ robot_->getGravity(); /*friction compensation?*/
}


Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{
    return Eigen::VectorXd::Zero(robot_->getNrJnts());
}


//velocity_null_ctrl

KDL::JntArray KDLController::velocity_ctrl_null(const Eigen::Vector3d& p_des,
                                                double Kp,
                                                double lambda,
                                                const Eigen::VectorXd& q_min,
                                                const Eigen::VectorXd& q_max)
{
    const int n = robot_->getNrJnts();

    Eigen::VectorXd q_curr = robot_->getJntValues();

    KDL::Frame T_be = robot_->getEEFrame();   
    Eigen::Vector3d p_ee(T_be.p.x(), T_be.p.y(), T_be.p.z());

    // position error
    Eigen::Vector3d e_p = p_des - p_ee;

    // only the linear part of the jacobian
    const Eigen::MatrixXd J6 = robot_->getEEJacobian().data;
    Eigen::MatrixXd J = J6.topRows(3);

    Eigen::MatrixXd J_pinv = pseudoinverse(J);

    Eigen::Vector3d u_cart = Kp * e_p;
    Eigen::VectorXd qdot_task = J_pinv * u_cart;   // Nx1

    // null-space: qdot0
    Eigen::VectorXd qdot0 = Eigen::VectorXd::Zero(n);
    for (int i = 0; i < n; ++i)
    {
        const double qmin = q_min(i);
        const double qmax = q_max(i);
        const double span = std::max(1e-9, qmax - qmin);
        const double denom = std::max(1e-12, lambda * span * span);
        qdot0(i) = ((qmax + qmin) - 2.0 * q_curr(i)) / denom;
    }

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd N = I - (J_pinv * J);

    // final command
    Eigen::VectorXd qdot_cmd_vec = qdot_task + N * qdot0;

    KDL::JntArray q_dot_cmd;
    q_dot_cmd.resize(n);
    q_dot_cmd.data = qdot_cmd_vec;
    return q_dot_cmd;
}

//Vision control

KDL::JntArray KDLController::vision_ctrl(const geometry_msgs::msg::PoseStamped& aruco_pose,
                                         double Kp,
                                         double lambda,
                                         const Eigen::VectorXd& q_min,
                                         const Eigen::VectorXd& q_max)
{
    const int n = robot_->getNrJnts();
    KDL::JntArray q_dot_cmd;
    q_dot_cmd.resize(n);

    // --- Implementazione Equazioni 3, 4, 5 ---

    // 1. Ottieni P_o (Posizione oggetto nel frame camera)
    //    ASSUNZIONE: Il topic /aruco_pose pubblica la posa del tag
    //    rispetto al frame della camera (es. camera_optical_link)
    Eigen::Vector3d P_o; 
    P_o << aruco_pose.pose.position.x, 
           aruco_pose.pose.position.y, 
           aruco_pose.pose.position.z;
    
    // 2. Calcola 's' (Eq. 4)
    double z = P_o.norm(); // z = ||^c P_o||
    if (z < 1e-6) // Evita divisione per zero
    {
        q_dot_cmd.data.setZero();
        return q_dot_cmd;
    }
    Eigen::Vector3d s = P_o / z; // s = P_o / ||P_o||

    // 3. Definisci il target 's_d'
    //    Come da compito, s_d = [0, 0, 1] (il tag è al centro, lungo l'asse Z)
    Eigen::Vector3d s_d(0.0, 0.0, 1.0);

    // 4. Calcola l'errore della feature
    Eigen::Vector3d e_s = s_d - s;

    // 5. Costruisci L(s) (Eq. 5)
    Eigen::MatrixXd Ls = Eigen::MatrixXd::Zero(3, 6);
    Eigen::Matrix3d I_3x3 = Eigen::Matrix3d::Identity();
    // Parte sinistra: -(1/z) * (I - s*s^T)
    Ls.block<3, 3>(0, 0) = (-1.0/z) * (I_3x3 - s * s.transpose());
    // Parte destra: S(s) (matrice skew-symmetric)
    Eigen::Matrix3d S_s;
    S_s <<  0.0,  -s(2),  s(1),
            s(2),   0.0, -s(0),
           -s(1),  s(0),  0.0;
    Ls.block<3, 3>(0, 3) = S_s;
    
    // 6. Ottieni J_c (Jacobiana della Camera nel frame Camera)
    
    // 6.a Ottieni J_e (Jacobiana EE nel frame Base)
    //     Grazie alla correzione in 1c.cpp (robot_->addEE(T_ee_camera)),
    //     questa è già la Jacobiana del *frame ottico*, ma espressa
    //     ancora nel frame /world (o /base). Quindi questa è ^B J_c
    const Eigen::MatrixXd J_e_base = robot_->getEEJacobian().data; // (6xN)

    // 6.b Ottieni la Posa della Camera nel frame Base (^B T_c)
    KDL::Frame T_base_camera = robot_->getEEFrame();

    // 6.c Estrai la rotazione da Base a Camera (^B R_c)
    Eigen::Matrix3d R_base_camera;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R_base_camera(i, j) = T_base_camera.M(i, j);

    // 6.d Calcola la rotazione da Camera a Base (^C R_B)
    Eigen::Matrix3d R_camera_base = R_base_camera.transpose();

    // 6.e Costruisci la matrice 6x6 di "cambio frame" per il Twist
    Eigen::MatrixXd V_base_to_camera = Eigen::MatrixXd::Zero(6, 6);
    V_base_to_camera.block<3, 3>(0, 0) = R_camera_base;
    V_base_to_camera.block<3, 3>(3, 3) = R_camera_base;
    
    // 6.f Calcola J_c ! (^C J_c = V * ^B J_c)
    Eigen::MatrixXd J_c = V_base_to_camera * J_e_base; // (6x6) * (6xN) = 6xN

    // 7. Calcola la Jacobiana dell'Interazione (Ls * Jc)
    Eigen::MatrixXd J_interaction = Ls * J_c; // (3x6) * (6xN) = 3xN

    // 8. Calcola il task primario (Eq. 3, parte 1)
    Eigen::MatrixXd J_pinv = pseudoinverse(J_interaction);
    Eigen::VectorXd qdot_task = J_pinv * (Kp * e_s); // (Nx3) * (3x1) = Nx1

    // 9. Calcola il task secondario (Null-space, Eq. 3, parte 2)
    Eigen::VectorXd q_curr = robot_->getJntValues();
    Eigen::VectorXd qdot0 = Eigen::VectorXd::Zero(n);
    for (int i = 0; i < n; ++i)
    {
        const double qmin = q_min(i);
        const double qmax = q_max(i);
        const double span = std::max(1e-9, qmax - qmin);
        const double denom = std::max(1e-12, lambda * span * span);
        qdot0(i) = ((qmax + qmin) - 2.0 * q_curr(i)) / denom;
    }

    // 10. Calcola il Proiettore Nullo (N)
    Eigen::MatrixXd I_NxN = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd N = I_NxN - (J_pinv * J_interaction);

    // 11. Calcola il comando finale (Eq. 3)
    Eigen::VectorXd qdot_cmd_vec = qdot_task + N * qdot0;

    // Impacchetta e restituisci
    q_dot_cmd.data = qdot_cmd_vec;
    return q_dot_cmd;
}