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
                           
    Eigen::VectorXd idCntr_euler(KDL::Frame &_desPos,
		                  KDL::Twist &_desVel,
		                  KDL::Twist &_desAcc,
		                  double _Kpp,
		                  double _Kpo,
		                  double _Kdp,
		                  double _Kdo);	

    Eigen::VectorXd idCntr(KDL::Vector &_desPos_p,
                           Eigen::Matrix<double,3,1> &_e_or,   
                           KDL::Twist &_desVel,
                           KDL::Twist &_desAcc,
                           double _Kpp,
                           double _Kpo,
                           double _Kdp,
                           double _Kdo);
    
private:

    KDLRobot* robot_;

};




#endif
