#include "kdl_control.h"
#include <iostream>


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

}

Eigen::VectorXd KDLController::velocity_ctrl_null(KDL::Frame &xd_,
                                                  double Kp_)
{

    // ottieni lo stato del robot
    KDL::Frame      x = robot_->getEEFrame();           // frame EE risp al world
    Eigen::MatrixXd J = robot_->getEEJacobian().data;   // jacobiano nel world
    int n = J.cols();
     
    Eigen::MatrixXd J_pos = J.topRows(3);          	// 3 x n: considero solo la parte lineare del Jacobiano

    // calcola l'errore
    Eigen::Vector3d ep_ = computeLinearError(toEigen(xd_.p), toEigen(x.p));
    Eigen::Vector3d eo_ = computeOrientationError(toEigen(xd_.M), toEigen(x.M));
    Eigen::VectorXd e_(6);
    e_ << ep_, eo_;

    // calcolo pseudoinversa
    Eigen::MatrixXd J_pinv = pseudoinverse(J);
    
    // calcolo del nullspace:
    double lambda = 0.5;                                // fattore di scala, regola la forza di "repulsione"
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd _qdot0(n);
    Eigen::MatrixXd q_limits = robot_->getJntLimits();  // col(0)=q_min, col(1)=q_max

    for (int i = 0; i < n; i++) {

        double qi = q(i);
        double qi_min = q_limits(i,0);
        double qi_max = q_limits(i,1);
        double range = qi_max - qi_min;
        double dist_from_max = qi_max - qi;
        double dist_from_min = qi - qi_min;
        
        if (dist_from_max < 0.01) dist_from_max = 0.01;
        if (dist_from_min < 0.01) dist_from_min = 0.01;
        
        double numerator = range * range;
        double denominator = dist_from_max * dist_from_min;

        _qdot0(i) = - (1.0 / lambda) * numerator * (2.0 * qi - qi_max - qi_min) / (denominator * denominator);

    }

    // primo termine
    Eigen::VectorXd v1 = J_pinv * (Kp_ * e_);
    
    // secondo termine, nullspace
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd NullProjector = I - J_pinv * J; 		// n x n
    Eigen::VectorXd v2 = NullProjector * _qdot0;      		// n x 1
    
    
    return v1+v2;
}