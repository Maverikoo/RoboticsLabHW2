#include "kdl_control.h"
#include "utils.h"

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
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis() + robot_->getGravity() /*friction compensation?*/;
}

Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{
    // Funzione non implementata (come nell'originale)
    return Eigen::VectorXd::Zero(robot_->getNrJnts());
}

// --- NUOVA FUNZIONE: Controllore di velocità standard (Pseudoinversa) ---
KDL::JntArray KDLController::velocity_ctrl(const Eigen::VectorXd& cartvel)
{
    // Questa è la logica "originale" che era in ros2_kdl_node_1b.cpp
    
    // Calcola la Jacobiana COMPLETA (6xN)
    Eigen::MatrixXd J = robot_->getEEJacobian().data;
    
    // Calcola la pseudoinversa
    Eigen::MatrixXd J_pinv = pseudoinverse(J);
    
    // Prepara il JntArray di output
    KDL::JntArray q_dot_cmd;
    q_dot_cmd.resize(robot_->getNrJnts());
    
    // Calcola il comando di velocità dei giunti
    // q_dot = J_pinv * x_dot_des
    q_dot_cmd.data = J_pinv * cartvel; 
    
    return q_dot_cmd;
}


// --- FUNZIONE MODIFICATA: Controllore con spazio nullo (6-DOF) ---
KDL::JntArray KDLController::velocity_ctrl_null(const Eigen::VectorXd& cartvel,
                                                double lambda,
                                                const Eigen::VectorXd& q_min,
                                                const Eigen::VectorXd& q_max)
{
    // read current state
    Eigen::VectorXd q_curr = robot_->getJntValues();
    int n = robot_->getNrJnts();

    // Calcola la Jacobiana COMPLETA (6xN)
    Eigen::MatrixXd J = robot_->getEEJacobian().data;
    
    // Calcola la pseudoinversa
    Eigen::MatrixXd J_pinv = pseudoinverse(J); 


    // --- Calcolo del termine di velocità nello spazio nullo (q0_dot) ---
    // (Questa è la tua logica originale per evitare i limiti)
    Eigen::VectorXd q0_dot = Eigen::VectorXd::Zero(n);
    for (int i = 0; i < n; ++i)
    {
        double q = q_curr(i);
        double q_max_i = q_max(i);
        double q_min_i = q_min(i);

        double q_range_sq = (q_max_i - q_min_i) * (q_max_i - q_min_i);
        double den_term = (q_max_i - q) * (q - q_min_i);
        double num_term = (2 * q - q_max_i - q_min_i);


        double epsilon = 1e-6; 
        if (std::abs(den_term) < epsilon) { // Evita divisione per zero
             q0_dot(i) = 0.0;
        } else {
            // Formula per il potenziale repulsivo (gradiente)
            q0_dot(i) = (q_range_sq / lambda) * (num_term / (den_term * den_term));
        }
    }

    // Calcola la matrice di proiezione nello spazio nullo (N)
    Eigen::MatrixXd I_n = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd N = I_n - J_pinv * J;

    // --- Formula finale del controllore ---
    // q_dot = (soluzione primaria) + (soluzione secondaria/spazio nullo)
    // q_dot = J_pinv * x_dot_des   + N * q0_dot
    
    // Prepara il JntArray di output
    KDL::JntArray q_dot_cmd;
    q_dot_cmd.resize(n);
    
    // Assegna il risultato
    q_dot_cmd.data = J_pinv * cartvel + N * q0_dot;

    return q_dot_cmd;
}