#include "kdl_ros_control/kdl_planner.h"

KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
}

KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    trajDuration_ = _trajDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, double _trajRadius)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajRadius_ = _trajRadius;
}

KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, double _trajRadius)
{
    trajDuration_ = _trajDuration;
    trajInit_ = _trajInit;
    trajRadius_ = _trajRadius;
}

void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                            double _radius, double _eqRadius
                                            )
{
    path_ = new KDL::Path_RoundedComposite(_radius,_eqRadius,new KDL::RotationalInterpolation_SingleAxis());

    for (unsigned int i = 0; i < _frames.size(); i++)
    {
        path_->Add(_frames[i]);
    }
    path_->Finish();

    velpref_->SetProfile(0,path_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_, velpref_);
}

void KDLPlanner::createCircPath(KDL::Frame &_F_start,
                                KDL::Vector &_V_centre,
                                KDL::Vector& _V_base_p,
                                KDL::Rotation& _R_base_end,
                                double alpha,
                                double eqradius
                                )
{
    KDL::RotationalInterpolation_SingleAxis* otraj;
    otraj = new KDL::RotationalInterpolation_SingleAxis();
    otraj->SetStartEnd(_F_start.M,_R_base_end);
    path_circle_ = new KDL::Path_Circle(_F_start,
                                        _V_centre,
                                        _V_base_p,
                                        _R_base_end,
                                        alpha,
                                        otraj,
                                        eqradius);
    velpref_->SetProfile(0,path_circle_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_circle_, velpref_);
}

KDL::Trajectory* KDLPlanner::getTrajectory()
{
	return traject_;
}

trajectory_point KDLPlanner::compute_trajectory(double time, int a)
{
    trajectory_point traj;

    double s,sd,sdd;
    
    //0-> Linear Trajectory TV, 1-> Linear Trajectory CP, 2 -> Circular Trajectory TV, 3-> Circular Trajectory CP
    
    switch(a){
        case 0:
        {
          trapezoidal_vel(time,s,sd,sdd);
          Eigen::Vector3d diff= trajEnd_ - trajInit_;

          traj.pos = trajInit_ + s*(diff);   //s in {0,1}   
          traj.vel = sd*(diff);
          traj.acc = sdd*(diff);
          //std::cout << "Linear Trajectory TV" << std::endl;
          break;
        }
        case 1:
        {  
          cubic_polynomial(time,s,sd,sdd);

          Eigen::Vector3d diff= trajEnd_ - trajInit_;

          traj.pos = trajInit_ + s*(diff);   //s in {0,1}   
          traj.vel = sd*(diff);
          traj.acc = sdd*(diff);
          //std::cout << "Linear Trajectory CP" << std::endl;
          break;
        }
        case 2:
        {
          trapezoidal_vel(time,s,sd,sdd);

          Eigen::Vector3d rot_component(0.0,-trajRadius_*std::cos(2*KDL::PI*s),-trajRadius_*std::sin(2*KDL::PI*s));
          Eigen::Vector3d centr_acc(0.0,4*std::pow(KDL::PI,2)*trajRadius_*std::pow(sd,2)*std::cos(2*KDL::PI*s),4*std::pow(KDL::PI,2)*trajRadius_*std::pow(sd,2)*std::sin(2*KDL::PI*s));
          Eigen::Vector3d tang_acc(0,2*KDL::PI*trajRadius_*sdd*std::sin(2*KDL::PI*s),-2*KDL::PI*trajRadius_*sdd*std::cos(2*KDL::PI*s));

          traj.pos = trajInit_ + rot_component;
          traj.vel = Eigen::Vector3d(0.0,2*KDL::PI*trajRadius_*sd*std::sin(2*KDL::PI*s),-2*KDL::PI*trajRadius_*sd*std::cos(2*KDL::PI*s));
          traj.acc = centr_acc + tang_acc;
          //std::cout << "Circular Trajectory TV" << std::endl;
          break;
        }   
        case 3:
        {
          cubic_polynomial(time,s,sd,sdd);

          Eigen::Vector3d rot_component(0.0,-trajRadius_*std::cos(2*KDL::PI*s),-trajRadius_*std::sin(2*KDL::PI*s));
          Eigen::Vector3d centr_acc(0.0,4*std::pow(KDL::PI,2)*trajRadius_*std::pow(sd,2)*std::cos(2*KDL::PI*s),4*std::pow(KDL::PI,2)*trajRadius_*std::pow(sd,2)*std::sin(2*KDL::PI*s));
          Eigen::Vector3d tang_acc(0,2*KDL::PI*trajRadius_*sdd*std::sin(2*KDL::PI*s),-2*KDL::PI*trajRadius_*sdd*std::cos(2*KDL::PI*s));

          traj.pos = trajInit_ + rot_component;
          traj.vel = Eigen::Vector3d(0.0,2*KDL::PI*trajRadius_*sd*std::sin(2*KDL::PI*s),-2*KDL::PI*trajRadius_*sd*std::cos(2*KDL::PI*s));
          traj.acc = centr_acc + tang_acc;
          //std::cout << "Circular Trajectory CP" << std::endl;
          break;
        }
        default:
            std::cout << "Error planner selection"<<std::endl;
    }
 
  return traj;

}

void KDLPlanner::trapezoidal_vel(double t, double &s, double &sdot, double &sdotdot) 
{
  double scdd = -1.0/(std::pow(accDuration_,2)-(trajDuration_*accDuration_));

  if(t <= accDuration_)
  {
    s = 0.5*scdd*std::pow(t,2);
    sdot = scdd*t;
    sdotdot = scdd;
  }
  else if(t <= trajDuration_- accDuration_)
  {
    s = scdd*accDuration_*(t-accDuration_/2); 
    sdot = scdd*accDuration_;
    sdotdot = 0.0;
  }
  else
  {
    s = 1 - 0.5*scdd*std::pow(trajDuration_-t,2);
    sdot = scdd*(trajDuration_-t);
    sdotdot = -scdd;
  }

  return;
}

void KDLPlanner::cubic_polynomial(double t, double &s,double &sdot,double &sdotdot){
   
  double a0,a1,a2,a3;

  a0 = 0.0;
  a1 = 0.0;
  a2 = 3/std::pow(trajDuration_,2);
  a3 = -2/std::pow(trajDuration_,3);

  s = a3*std::pow(t,3)+a2*std::pow(t,2)+a1*t+a0;   
  sdot = 3*a3*std::pow(t,2)+2*a2*t+a1;
  sdotdot = 6*a3*t+2*a2;

  return;   
}
