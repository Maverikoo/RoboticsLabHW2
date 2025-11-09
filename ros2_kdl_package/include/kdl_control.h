#ifndef KDLControl
#define KDLControl

#include "Eigen/Dense"
#include "kdl_robot.h"
#include "utils.h"

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
                           
  
    // --- FIRME AGGIORNATE PER IL CONTROLLO DI VELOCITÀ ---

    /*
     * @brief Controllore di velocità standard (Pseudoinversa).
     * @param cartvel Il vettore 6D di velocità cartesiane desiderate (Twist).
     * @return KDL::JntArray Le velocità dei giunti calcolate.
     */
    KDL::JntArray velocity_ctrl(const Eigen::VectorXd& cartvel);

    /*
     * @brief Controllore di velocità con gestione dello spazio nullo (per evitare i limiti).
     * @param cartvel Il vettore 6D di velocità cartesiane desiderate (Twist).
     * @param lambda Guadagno per il termine di spazio nullo.
     * @param q_min Vettore dei limiti inferiori dei giunti.
     * @param q_max Vettore dei limiti superiori dei giunti.
     * @return KDL::JntArray Le velocità dei giunti calcolate.
     */
    KDL::JntArray velocity_ctrl_null(const Eigen::VectorXd& cartvel,
                                     double lambda,
                                     const Eigen::VectorXd& q_min,
                                     const Eigen::VectorXd& q_max); 

private:

    KDLRobot* robot_;

};

#endif