#include "kdl_ros_control/kdl_control.h"

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
            + robot_->getCoriolis() + robot_->getGravity();
}

Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{
    
    // calculate gain matrices
    Eigen::Matrix<double,6,6> Kp = Eigen::MatrixXd::Zero(6, 6);
    Eigen::Matrix<double,6,6> Kd = Eigen::MatrixXd::Zero(6, 6);

    Kp.block(0,0,3,3) = _Kpp*Eigen::Matrix3d::Identity();
    Kp.block(3,3,3,3) = _Kpo*Eigen::Matrix3d::Identity();
    Kd.block(0,0,3,3) = _Kdp*Eigen::Matrix3d::Identity();
    Kd.block(3,3,3,3) = _Kdo*Eigen::Matrix3d::Identity();
   // Kp(2,2) = _Kpp + 170;                                        //gain offset along z-axis 
   // Kp(3,3)=_Kpo+30; 

    //DEBUG
    //std::cout << "Kp: " << std::endl << Kp << std::endl;

    // read current state
    Eigen::Matrix<double,6,7> J(robot_->getEEJacobian().data);
    Eigen::Matrix<double,7,7> I = Eigen::Matrix<double,7,7>::Identity();
    Eigen::Matrix<double,7,7> M = robot_->getJsim();
    
    // Eigen::Matrix<double,7,6> Jpinv = weightedPseudoInverse(M,J);
       Eigen::Matrix<double,7,6> Jpinv = pseudoinverse(J);

    // position
    Eigen::Vector3d p_d(_desPos.p.data);
    Eigen::Vector3d p_e(robot_->getEEFrame().p.data); 
    
    Eigen::Matrix<double,3,3,Eigen::RowMajor> R_d(_desPos.M.data); 
    Eigen::Matrix<double,3,3,Eigen::RowMajor> R_e(robot_->getEEFrame().M.data);
   
    R_d = matrixOrthonormalization(R_d);
    R_e = matrixOrthonormalization(R_e);

    // velocity
    Eigen::Vector3d dot_p_d(_desVel.vel.data);
    Eigen::Vector3d dot_p_e(robot_->getEEVelocity().vel.data);

    Eigen::Vector3d omega_d(_desVel.rot.data);
    Eigen::Vector3d omega_e(robot_->getEEVelocity().rot.data);
   
    // acceleration
    Eigen::Matrix<double,6,1> dot_dot_x_d=Eigen::MatrixXd::Zero(6, 1);

    Eigen::Matrix<double,3,1> dot_dot_p_d(_desAcc.vel.data);
    Eigen::Matrix<double,3,1> dot_dot_r_d(_desAcc.rot.data);

    // compute linear errors
    Eigen::Matrix<double,3,1> e_p = computeLinearError(p_d,p_e);

    Eigen::Matrix<double,3,1> dot_e_p = computeLinearError(dot_p_d,dot_p_e);

    // compute orientation errors
    Eigen::Matrix<double,3,1> e_o = computeOrientationError(R_d,R_e); 
    Eigen::Matrix<double,3,1> dot_e_o =computeOrientationVelocityError(omega_d, omega_e, R_d, R_e);    
    
    Eigen::Matrix<double,6,1> x_tilde = Eigen::MatrixXd::Zero(6, 1);
    Eigen::Matrix<double,6,1> dot_x_tilde = Eigen::MatrixXd::Zero(6, 1);
    x_tilde << e_p, e_o;
    dot_x_tilde << dot_e_p, dot_e_o;
    dot_dot_x_d << dot_dot_p_d, dot_dot_r_d;

    // null space control
    double cost=0.0;
    Eigen::VectorXd grad = gradientJointLimits(robot_->getJntValues(),robot_->getJntLimits(),cost);

////    std::cout << "---------------------" << std::endl;
////    std::cout << "p_d: " << std::endl << p_d << std::endl;
////    std::cout << "p_e: " << std::endl << p_e << std::endl;
////    std::cout << "dot_p_d: " << std::endl << dot_p_d << std::endl;
////    std::cout << "dot_p_e: " << std::endl << dot_p_e << std::endl;
////    std::cout << "R_d: " << std::endl << R_d << std::endl;
////    std::cout << "R_e: " << std::endl << R_e << std::endl;
////    std::cout << "omega_d: " << std::endl << omega_d << std::endl;
////    std::cout << "omega_e: " << std::endl << omega_e << std::endl;
////    std::cout << "x_tilde: " << std::endl << x_tilde << std::endl;
////    std::cout << "dot_x_tilde: " << std::endl << dot_x_tilde << std::endl;
////    std::cout << "jacobian: " << std::endl << robot_->getJacobian() << std::endl;
////    std::cout << "jpinv: " << std::endl << Jpinv << std::endl;
////    std::cout << "jsim: " << std::endl << robot_->getJsim() << std::endl;
////    std::cout << "c: " << std::endl << robot_->getCoriolis().transpose() << std::endl;
////    std::cout << "g: " << std::endl << robot_->getGravity().transpose() << std::endl;
////    std::cout << "q: " << std::endl << robot_->getJntValues().transpose() << std::endl;
////    std::cout << "Jac Dot qDot: " << std::endl << robot_->getJacDotqDot().transpose() << std::endl;
////    std::cout << "Jnt lmt cost: " << std::endl << cost << std::endl;
////    std::cout << "Jnt lmt gradient: " << std::endl << grad.transpose() << std::endl;
////    std::cout << "---------------------" << std::endl;

    // inverse dynamics
    Eigen::Matrix<double,6,1> y = Eigen::MatrixXd::Zero(6, 1);
    Eigen::Matrix<double,6,1> Jdqd(robot_->getEEJacDotqDot());
   
    y << dot_dot_x_d - Jdqd + Kd*dot_x_tilde + Kp*x_tilde; 

    //return M * (Jpinv*y  +(I-Jpinv*J)*(- 10*grad - 1*robot_->getJntVelocities()))+ robot_->getGravity() + robot_->getCoriolis();
    return M * (Jpinv*y + (I-Jpinv*J)*(-grad))+ robot_->getGravity() + robot_->getCoriolis();

}


Eigen::VectorXd KDLController::idCntr(KDL::Vector &_desPos_p,
                                      Eigen::Matrix<double,3,1> &_e_or,   
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{
    
    // calculate gain matrices
    Eigen::Matrix<double,6,6> Kp = Eigen::MatrixXd::Zero(6, 6);
    Eigen::Matrix<double,6,6> Kd = Eigen::MatrixXd::Zero(6, 6);

    Kp.block(0,0,3,3) = _Kpp*Eigen::Matrix3d::Identity();
    Kp.block(3,3,3,3) = _Kpo*Eigen::Matrix3d::Identity();
    Kd.block(0,0,3,3) = _Kdp*Eigen::Matrix3d::Identity();
    Kd.block(3,3,3,3) = _Kdo*Eigen::Matrix3d::Identity();
  
    //DEBUG
    //std::cout << "Kp: " << std::endl << Kp << std::endl;

    // read current state
    Eigen::Matrix<double,6,7> J(robot_->getEEJacobian().data);
    Eigen::Matrix<double,7,7> I = Eigen::Matrix<double,7,7>::Identity();
    Eigen::Matrix<double,7,7> M = robot_->getJsim();
    
    // Eigen::Matrix<double,7,6> Jpinv = weightedPseudoInverse(M,J);
       Eigen::Matrix<double,7,6> Jpinv = pseudoinverse(J);
           
    // position
    Eigen::Vector3d p_d(_desPos_p.data);
    Eigen::Vector3d p_e(robot_->getEEFrame().p.data); 
    
    // velocity
    Eigen::Vector3d dot_p_d(_desVel.vel.data);
    Eigen::Vector3d dot_p_e(robot_->getEEVelocity().vel.data);

    Eigen::Vector3d omega_d(_desVel.rot.data);
    Eigen::Vector3d omega_e(robot_->getEEVelocity().rot.data);
   
    // acceleration
    Eigen::Matrix<double,6,1> dot_dot_x_d=Eigen::MatrixXd::Zero(6, 1);

    Eigen::Matrix<double,3,1> dot_dot_p_d(_desAcc.vel.data);
    Eigen::Matrix<double,3,1> dot_dot_r_d(_desAcc.rot.data);

    // compute linear errors
    Eigen::Matrix<double,3,1> e_p = computeLinearError(p_d,p_e);
    Eigen::Matrix<double,3,1> dot_e_p = computeLinearError(dot_p_d,dot_p_e);

    // compute orientation errors
   Eigen::Matrix<double,3,1> dot_e_o =-omega_e;// computeOrientationVelocityError(omega_d, omega_e, R_d, R_e);    

    Eigen::Matrix<double,6,1> x_tilde = Eigen::MatrixXd::Zero(6, 1);
    Eigen::Matrix<double,6,1> dot_x_tilde = Eigen::MatrixXd::Zero(6, 1);
    x_tilde << e_p, _e_or;
    dot_x_tilde << dot_e_p, dot_e_o;
    dot_dot_x_d << dot_dot_p_d, dot_dot_r_d;

    // null space control
    double cost=0.0;
    Eigen::VectorXd grad = gradientJointLimits(robot_->getJntValues(),robot_->getJntLimits(),cost);

////    std::cout << "---------------------" << std::endl;
////    std::cout << "p_d: " << std::endl << p_d << std::endl;
////    std::cout << "p_e: " << std::endl << p_e << std::endl;
////    std::cout << "dot_p_d: " << std::endl << dot_p_d << std::endl;
////    std::cout << "dot_p_e: " << std::endl << dot_p_e << std::endl;
////    std::cout << "R_d: " << std::endl << R_d << std::endl;
////    std::cout << "R_e: " << std::endl << R_e << std::endl;
////    std::cout << "omega_d: " << std::endl << omega_d << std::endl;
////    std::cout << "omega_e: " << std::endl << omega_e << std::endl;
////    std::cout << "x_tilde: " << std::endl << x_tilde << std::endl;
////    std::cout << "dot_x_tilde: " << std::endl << dot_x_tilde << std::endl;
////    std::cout << "jacobian: " << std::endl << robot_->getJacobian() << std::endl;
////    std::cout << "jpinv: " << std::endl << Jpinv << std::endl;
////    std::cout << "jsim: " << std::endl << robot_->getJsim() << std::endl;
////    std::cout << "c: " << std::endl << robot_->getCoriolis().transpose() << std::endl;
////    std::cout << "g: " << std::endl << robot_->getGravity().transpose() << std::endl;
////    std::cout << "q: " << std::endl << robot_->getJntValues().transpose() << std::endl;
////    std::cout << "Jac Dot qDot: " << std::endl << robot_->getJacDotqDot().transpose() << std::endl;
////    std::cout << "Jnt lmt cost: " << std::endl << cost << std::endl;
////    std::cout << "Jnt lmt gradient: " << std::endl << grad.transpose() << std::endl;
////    std::cout << "---------------------" << std::endl;

    // inverse dynamics
    Eigen::Matrix<double,6,1> y = Eigen::MatrixXd::Zero(6, 1);
    Eigen::Matrix<double,6,1> Jdqd(robot_->getEEJacDotqDot());
   
    y << dot_dot_x_d - Jdqd + Kd*dot_x_tilde + Kp*x_tilde; 

    //return M * (Jpinv*y  +(I-Jpinv*J)*(- 10*grad - 1*robot_->getJntVelocities()))+ robot_->getGravity() + robot_->getCoriolis();
    return M * (Jpinv*y + (I-Jpinv*J)*(-grad))+ robot_->getGravity() + robot_->getCoriolis();
   
}


Eigen::VectorXd KDLController::idCntr_euler(KDL::Frame &_desPos,
                                            KDL::Twist &_desVel,
                                            KDL::Twist &_desAcc,
                                            double _Kpp, double _Kpo,
                                            double _Kdp, double _Kdo) {
    // calculate gain matrices
    Eigen::Matrix<double,6,6> Kp = Eigen::MatrixXd::Zero(6,6);
    Eigen::Matrix<double,6,6> Kd = Eigen::MatrixXd::Zero(6,6);
    Kp.block(0,0,3,3) = _Kpp*Eigen::Matrix3d::Identity();
    Kp.block(3,3,3,3) = _Kpo*Eigen::Matrix3d::Identity();
    Kd.block(0,0,3,3) = _Kdp*Eigen::Matrix3d::Identity();
    Kd.block(3,3,3,3) = _Kdo*Eigen::Matrix3d::Identity();
    //Kp(2,2) = Kp(2,2) + 150;
    //Kp(1,1) = Kp(1,1) + 40;

    // read current state
    Eigen::Matrix<double,7,1> q = robot_->getJntValues();
    Eigen::Matrix<double,7,1> dq = robot_->getJntVelocities();
    Eigen::Matrix<double,7,7> B = robot_->getJsim();
    Eigen::Matrix<double,7,7> I = Eigen::Matrix<double,7,7>::Identity();
    Eigen::Matrix<double,6,7> J = robot_->getEEJacobian().data;
    


///// x_tilde computation //////
    // position
    Eigen::Vector3d p_d(_desPos.p.data);
    Eigen::Vector3d p_e(robot_->getEEFrame().p.data);
    Eigen::Vector3d e_p = p_d - p_e;
    
    // orientation
    Eigen::Vector3d euler_d(0,0,0);           
    Eigen::Vector3d euler_e(0,0,0);
    Eigen::Vector3d e_euler(0,0,0);
    double alpha_e = 0, beta_e = 0, gamma_e = 0;
    double alpha_d = 0, beta_d = 0, gamma_d = 0;

    Eigen::Matrix<double,3,3,Eigen::RowMajor> R_d(_desPos.M.data);
    Eigen::Matrix<double,3,3,Eigen::RowMajor> R_e(robot_->getEEFrame().M.data);
    R_d = matrixOrthonormalization(R_d);
    R_e = matrixOrthonormalization(R_e);

    // conversion from Eigen::Matrix to KDL::Rotation (needed for euler angle extraction)
    KDL::Rotation R_d2 = KDL::Rotation::Identity();     
    KDL::Rotation R_e2 = KDL::Rotation::Identity();
    for (int i = 0; i < 3; ++i) {       
        for (int j = 0; j < 3; ++j)
            R_d2(i,j) = R_d(i,j);
    }
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j)
            R_e2(i,j) = R_e(i,j);
    }

    R_d2.GetEulerZYZ(alpha_d,beta_d,gamma_d);
    R_e2.GetEulerZYZ(alpha_e,beta_e,gamma_e);
    euler_d << alpha_d,beta_d,gamma_d;
    euler_e << alpha_e,beta_e,gamma_e;
    e_euler = euler_d - euler_e;
    
    // x_tilde
    Eigen::Matrix<double,6,1> x_tilde;
    x_tilde.setZero();
    x_tilde << e_p, e_euler;



////// dot_x_tilde computation //////
    // conversion from KDL::Twist to Eigen::Matrix
    Eigen::Matrix<double,6,1> ve_d = Eigen::Matrix<double,6,1>::Zero();
    ve_d.head(3) << _desVel.vel.x(), _desVel.vel.y(), _desVel.vel.z();
    ve_d.tail(3) << _desVel.rot.x(), _desVel.rot.y(), _desVel.rot.z();
    Eigen::Matrix<double,6,1> ve_e = Eigen::Matrix<double,6,1>::Zero();
    ve_e.head(3) << robot_->getEEVelocity().vel.x(), robot_->getEEVelocity().vel.y(), robot_->getEEVelocity().vel.z();
    ve_e.tail(3) << robot_->getEEVelocity().rot.x(), robot_->getEEVelocity().rot.y(), robot_->getEEVelocity().rot.z();
    
    // TA_d
    Eigen::MatrixXd s_TA_d = Eigen::MatrixXd::Zero(6,6);
    s_TA_d.block<3,3>(0,0) = Eigen::Matrix<double, 3, 3>::Identity();
    Eigen::Matrix3d s_T_d = Eigen::Matrix3d::Zero();
    eul_kin_ZYZ(beta_d, alpha_d, s_T_d);
    s_TA_d.block<3,3>(3,3) = s_T_d;
    
    // TA_e
    Eigen::MatrixXd s_TA_e = Eigen::MatrixXd::Zero(6,6);
    s_TA_e.block<3,3>(0,0) = Eigen::Matrix<double, 3, 3>::Identity();
    Eigen::Matrix3d s_T_e = Eigen::Matrix3d::Zero();
    eul_kin_ZYZ(beta_e, alpha_e, s_T_e);
    s_TA_e.block<3,3>(3,3) = s_T_e;
    
    // dot_x_d, dot_x_e
    Eigen::Matrix<double,6,1> dot_x_d = s_TA_d.inverse() * ve_d;
    Eigen::Matrix<double,6,1> dot_x_e = s_TA_e.inverse() * ve_e;
    Eigen::Matrix<double,6,1> dot_x_tilde;
    dot_x_tilde.setZero();
    dot_x_tilde << (dot_x_d - dot_x_e);

    

////// s_JA_ee_dot computation//////
    // JA_d
    Eigen::MatrixXd s_JA_ee_d(6,7);
    s_JA_ee_d.setZero();
    s_JA_ee_d = s_TA_d.inverse() * J;
    
    // JA_e
    Eigen::MatrixXd s_JA_ee_e(6,7);
    s_JA_ee_e.setZero();
    s_JA_ee_e = s_TA_e.inverse() * J;
    
    // s_TA_dot
    Eigen::MatrixXd s_TA_dot_ = Eigen::MatrixXd::Zero(6,6);
    Eigen::Matrix3d s_T_dot = Eigen::Matrix3d::Zero();
    eul_kin_ZYZ_dot(beta_e, alpha_e, dot_x_e(4), dot_x_e(3), s_T_dot);
    s_TA_dot_.block<3,3>(3,3) = s_T_dot;
    
    // s_JA_ee_dot
    Eigen::MatrixXd s_JA_ee_dot(6,7);
    Eigen::Matrix<double,6,7> J_dot = robot_->getEEJacDot().data;
    s_JA_ee_dot.setZero();
    s_JA_ee_dot = s_TA_e.inverse()*(J_dot - s_TA_dot_*s_JA_ee_e);
    


////// dot_dot_x_d computation //////
    Eigen::Matrix<double,6,1> dot_ve_d;     // conversion from KDL::Twist to Eigen::Matrix
    dot_ve_d.setZero();
    dot_ve_d.head(3) << _desAcc.vel.x(), _desAcc.vel.y(), _desAcc.vel.z();
    dot_ve_d.tail(3) << _desAcc.rot.x(), _desAcc.rot.y(), _desAcc.rot.z();
    Eigen::Matrix<double,6,1> dot_dot_x_d;
    dot_dot_x_d.setZero();
    dot_dot_x_d = s_TA_d.inverse()*(dot_ve_d - s_TA_dot_*dot_x_d);



////// tau computation //////
    Eigen::Matrix<double,6,1> y;
    y.setZero();
    y <<  dot_dot_x_d + Kd*dot_x_tilde + Kp*x_tilde; //- s_JA_ee_dot*dq;

    // JApinv    
    Eigen::Matrix<double,7,6> JApinv = weightedPseudoInverse(I,s_JA_ee_e);
    
    // null space control
    double cost;
    Eigen::VectorXd grad = Eigen::VectorXd::Zero(7);
    grad = gradientJointLimits(robot_->getJntValues(),robot_->getJntLimits(),cost);
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(7);
    tau = B*(JApinv*y + (I-JApinv*s_JA_ee_e)*(-grad- 1*robot_->getJntVelocities())) + robot_->getGravity() + robot_->getCoriolis();

    return tau;
}

