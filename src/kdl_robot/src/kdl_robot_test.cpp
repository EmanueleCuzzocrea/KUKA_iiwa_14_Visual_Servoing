#include "kdl_ros_control/kdl_robot.h"
#include "kdl_ros_control/kdl_control.h"
#include "kdl_ros_control/kdl_planner.h"

#include "kdl_parser/kdl_parser.hpp"
#include "urdf/model.h"
#include <std_srvs/Empty.h>
#include "eigen_conversions/eigen_kdl.h"

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "gazebo_msgs/SetModelConfiguration.h"


// Global variables
std::vector<double> jnt_pos(7,0.0), jnt_vel(7,0.0),init_jnt_pos(7,0.0),aruco_pose(7,0.0); 
bool robot_state_available = false, aruco_pose_available = false;

// Functions
KDLRobot createRobot(std::string robot_string)
{
    KDL::Tree robot_tree;
    urdf::Model my_model;
    if (!my_model.initFile(robot_string))
    {
        printf("Failed to parse urdf robot model \n");
    }
    if (!kdl_parser::treeFromUrdfModel(my_model, robot_tree))
    {
        printf("Failed to construct kdl tree \n");
    }
    
    KDLRobot robot(robot_tree);
    return robot;
}

void jointStateCallback(const sensor_msgs::JointState & msg)
{
    robot_state_available = true;
    jnt_pos.clear();
    jnt_vel.clear();
    for (int i = 0; i < msg.position.size(); i++)
    {
        jnt_pos.push_back(msg.position[i]);
        jnt_vel.push_back(msg.velocity[i]);
    }
}
void arucoPoseCallback(const geometry_msgs::PoseStamped & msg)
{
    aruco_pose_available = true;
    aruco_pose.clear();
    aruco_pose.push_back(msg.pose.position.x);
    aruco_pose.push_back(msg.pose.position.y);
    aruco_pose.push_back(msg.pose.position.z);
    aruco_pose.push_back(msg.pose.orientation.x);
    aruco_pose.push_back(msg.pose.orientation.y);
    aruco_pose.push_back(msg.pose.orientation.z);
    aruco_pose.push_back(msg.pose.orientation.w);
}


// Main
int main(int argc, char **argv)
{
    if (argc < 2)
    {
        printf("Please, provide a path to a URDF file...\n");
        return 0;
    }

    // Init node
    ros::init(argc, argv, "kdl_ros_control_node");
    ros::NodeHandle n;

    // Rate
    ros::Rate loop_rate(500);

    // Subscribers
    ros::Subscriber joint_state_sub = n.subscribe("/iiwa/joint_states", 1, jointStateCallback);
    ros::Subscriber aruco_pose_sub = n.subscribe("/aruco_single/pose", 1, arucoPoseCallback);

    // Publishers
    ros::Publisher joint1_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_1_effort_controller/command", 1);
    ros::Publisher joint2_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_2_effort_controller/command", 1);
    ros::Publisher joint3_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_3_effort_controller/command", 1);
    ros::Publisher joint4_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_4_effort_controller/command", 1);
    ros::Publisher joint5_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_5_effort_controller/command", 1);
    ros::Publisher joint6_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_6_effort_controller/command", 1);
    ros::Publisher joint7_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_7_effort_controller/command", 1);    

    // Services
    ros::ServiceClient robot_set_state_srv = n.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");
    ros::ServiceClient pauseGazebo = n.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");

    // Set robot state
    gazebo_msgs::SetModelConfiguration robot_init_config;
    robot_init_config.request.model_name = "iiwa";
    robot_init_config.request.urdf_param_name = "robot_description";
    robot_init_config.request.joint_names.push_back("iiwa_joint_1");
    robot_init_config.request.joint_names.push_back("iiwa_joint_2");
    robot_init_config.request.joint_names.push_back("iiwa_joint_3");
    robot_init_config.request.joint_names.push_back("iiwa_joint_4");
    robot_init_config.request.joint_names.push_back("iiwa_joint_5");
    robot_init_config.request.joint_names.push_back("iiwa_joint_6");
    robot_init_config.request.joint_names.push_back("iiwa_joint_7");
    robot_init_config.request.joint_positions.push_back(0.0);
    robot_init_config.request.joint_positions.push_back(1.57);
    robot_init_config.request.joint_positions.push_back(-1.57);
    robot_init_config.request.joint_positions.push_back(-1.57);
    robot_init_config.request.joint_positions.push_back(1.57);
    robot_init_config.request.joint_positions.push_back(-1.57);
    robot_init_config.request.joint_positions.push_back(1.57);
    if(robot_set_state_srv.call(robot_init_config))
        ROS_INFO("Robot state set.");
    else
        ROS_INFO("Failed to set robot state.");


    // Messages
    std_msgs::Float64 tau1_msg, tau2_msg, tau3_msg, tau4_msg, tau5_msg, tau6_msg, tau7_msg;
    std_srvs::Empty pauseSrv;

    // Wait for robot and object state
    while (!(robot_state_available))
    {
        ROS_INFO_STREAM_ONCE("Robot/object state not available yet.");
        ROS_INFO_STREAM_ONCE("Please start gazebo simulation.");
        if (!(robot_set_state_srv.call(robot_init_config)))
            ROS_INFO("Failed to set robot state.");            
        
        ros::spinOnce();
    }

    // Create robot
    KDLRobot robot = createRobot(argv[1]);
    robot.update(jnt_pos, jnt_vel);
    int nrJnts = robot.getNrJnts();

    // Specify an end-effector 
    KDL::Frame ee_T_cam;
    ee_T_cam.M = KDL::Rotation::RotY(1.57)*KDL::Rotation::RotZ(-1.57);
    ee_T_cam.p = KDL::Vector(0,0,0.025);
    robot.addEE(ee_T_cam);

    // Joints
    KDL::JntArray qd(robot.getNrJnts()),dqd(robot.getNrJnts()),ddqd(robot.getNrJnts());
    qd.data.setZero();
    dqd.data.setZero();
    ddqd.data.setZero();

    // Torques
    Eigen::VectorXd tau;
    tau.resize(robot.getNrJnts());

    // Update robot
    robot.update(jnt_pos, jnt_vel);

    // Init controller
    KDLController controller_(robot);

    // EE's trajectory initial position
    KDL::Frame init_cart_pose = robot.getEEFrame(); //Fi
    Eigen::Vector3d init_position(init_cart_pose.p.data); //pdi

    // EE trajectory end position (needed only for linear trajectory)
    Eigen::Vector3d end_position; 
    end_position << init_cart_pose.p.x(), -init_cart_pose.p.y(), init_cart_pose.p.z();

    // Plan trajectory
    double traj_duration = 5, acc_duration = 1, t = 0.0, init_time_slot = 1.0, radius = 0.1;
    // double traj_duration = 8, acc_duration = 2, t = 0.0, init_time_slot = 2.0, radius = 0.1; // Euler angles parameters
    
    int a = 0;      //0-> Linear Trajectory TV, 1-> Linear Trajectory CP, 2 -> Circular Trajectory TV, 3-> Circular Trajectory CP

    KDLPlanner planner(traj_duration, acc_duration, init_position, end_position);    //0
    //KDLPlanner planner(traj_duration, init_position, end_position);                  //1
    //KDLPlanner planner(traj_duration, acc_duration, init_position,radius);           //2
    //KDLPlanner planner(traj_duration, init_position,radius);                         //3

    trajectory_point p = planner.compute_trajectory(t, a);

    // Retrieve initial simulation time
    ros::Time begin = ros::Time::now();
    ROS_INFO_STREAM_ONCE("Starting control loop ...");

    // Init trajectory
    KDL::Frame des_pose = KDL::Frame::Identity(); KDL::Twist des_cart_vel = KDL::Twist::Zero(), des_cart_acc = KDL::Twist::Zero();
   // des_pose.M = robot.getEEFrame().M;

    while ((ros::Time::now()-begin).toSec() < 2*traj_duration + init_time_slot)
    {
        if (robot_state_available && aruco_pose_available)
        {
            // Update robot
            robot.update(jnt_pos, jnt_vel);

            // Update time
            t = (ros::Time::now()-begin).toSec();
            std::cout << "time: " << t << std::endl;
   
            // compute current jacobians
            KDL::Jacobian J_cam = robot.getEEJacobian();
            KDL::Frame cam_T_object(KDL::Rotation::Quaternion(aruco_pose[3], aruco_pose[4], aruco_pose[5], aruco_pose[6]), KDL::Vector(aruco_pose[0], aruco_pose[1], aruco_pose[2]));

            // look at point: compute rotation error from angle/axis
            Eigen::Matrix<double,3,1> aruco_pos_n = toEigen(cam_T_object.p); //(aruco_pose[0],aruco_pose[1],aruco_pose[2]);
            aruco_pos_n.normalize(); //s
            
            Eigen::Vector3d r_o = skew(Eigen::Vector3d(0,0,1))*aruco_pos_n;
            double aruco_angle = std::acos(Eigen::Vector3d(0,0,1).dot(aruco_pos_n));
            KDL::Rotation Re = KDL::Rotation::Rot(KDL::Vector(r_o[0], r_o[1], r_o[2]), aruco_angle); 
        
        /*
            //double offset_or=0.5;
            Eigen::Matrix<double,3,1> e_o = computeOrientationError(toEigen(robot.getEEFrame().M*Re), toEigen(robot.getEEFrame().M));
            //Eigen::Matrix<double,3,1> e_o_w = computeOrientationError(toEigen(robot.getEEFrame().M*cam_T_object.M*KDL::Rotation::RotX(-3.14)*KDL::Rotation::RotZ(offset_or)), toEigen(robot.getEEFrame().M));   
            Eigen::Matrix<double,3,1> e_o_w = computeOrientationError(toEigen(robot.getEEFrame().M*cam_T_object.M*KDL::Rotation::RotX(-3.14)), toEigen(robot.getEEFrame().M));        
            //Eigen::Matrix<double,3,1> e_o_w = computeOrientationError(toEigen(init_cart_pose.M), toEigen(robot.getEEFrame().M));
 
            Eigen::Matrix<double,3,1> e_or; e_or << e_o_w[0], e_o[1], e_o[2];
        */
           
            // Extract desired pose
            des_cart_vel = KDL::Twist::Zero();
            des_cart_acc = KDL::Twist::Zero();
            if (t <= init_time_slot) // wait a second
            {
                p = planner.compute_trajectory(0.0, a);
            }
            else if(t > init_time_slot && t <= traj_duration + init_time_slot)
            {
                p = planner.compute_trajectory(t-init_time_slot, a);
                des_cart_vel = KDL::Twist(KDL::Vector(p.vel[0], p.vel[1], p.vel[2]),KDL::Vector::Zero());
                des_cart_acc = KDL::Twist(KDL::Vector(p.acc[0], p.acc[1], p.acc[2]),KDL::Vector::Zero());
            }
            else
            {
                ROS_INFO_STREAM_ONCE("trajectory terminated");
                break;
            }          
 
            // EE's trajectory position
            KDL::Frame pose = robot.getEEFrame();
            Eigen::Vector3d position_now(pose.p.data);
            Eigen::Vector3d epos = p.pos - position_now;
            
            //DEBUG
            //std::cout << "Position Error: " << std::endl << epos << std::endl;
            std::cout << "Norm position error: " << epos.norm() << std::endl;
            //std::cout << "Orientation Error e_o: " << std::endl << e_o << std::endl;
            //std::cout << "Orientation Error e_or: " << std::endl << e_or << std::endl;
            // std::cout << "jacobian: " << std::endl << robot.getEEJacobian().data << std::endl;
            // std::cout << "jsim: " << std::endl << robot.getJsim() << std::endl;
            // std::cout << "c: " << std::endl << robot.getCoriolis().transpose() << std::endl;
            // std::cout << "g: " << std::endl << robot.getGravity().transpose() << std::endl;
            // std::cout << "qd: " << std::endl << qd.data.transpose() << std::endl;
            // std::cout << "q: " << std::endl << robot.getJntValues().transpose() << std::endl;
            // std::cout << "tau: " << std::endl << tau.transpose() << std::endl;
            //std::cout << "desired_pose: " << std::endl << des_pose << std::endl;
            // std::cout << "current_pose: " << std::endl << robot.getEEFrame() << std::endl;

                      
            Eigen::Matrix<double,3,1> s_b = toEigen(robot.getEEFrame().p) + toEigen(robot.getEEFrame().M)*aruco_pos_n;
            Eigen::Matrix<double,3,1> camera_b = toEigen(robot.getEEFrame().p);
            double _a = s_b(2) - camera_b(2);
            double alpha = std::asin(_a); //s has norm = 1
            double beta = 1.57 - alpha;
            double _b = std::sin(beta); //w has norm = 1
            double _c = camera_b(2) - _b;
            double _d = std::cos(beta);
            double _e = _d/std::cos(alpha);
            
            Eigen::Matrix<double,3,1> w = _e*aruco_pos_n; // assigning x and y to w (the z component is corrected below)
            Eigen::Matrix<double,3,1> w_b = toEigen(robot.getEEFrame().p) + toEigen(robot.getEEFrame().M)*w;
            w_b(2) = _c;
            w= -(toEigen(robot.getEEFrame().M)).transpose()*toEigen(robot.getEEFrame().p) + (toEigen(robot.getEEFrame().M)).transpose()*w_b;
            w.normalize();

            Eigen::Matrix<double,3,1> u = skew(w)*aruco_pos_n;
            u.normalize();
            //std::cout << "s: " << std::endl << aruco_pos_n << std::endl;
            //std::cout << "s2: " << std::endl << w << std::endl;
            //std::cout << "s3: " << std::endl << u << std::endl;
            KDL::Rotation RefOrFrame(
                u(0), w(0), aruco_pos_n(0),
                u(1), w(1), aruco_pos_n(1),
                u(2), w(2), aruco_pos_n(2)
            );
             Eigen::MatrixXd des_pose_n(3,3);
    
            for (int i = 0; i < 3; ++i)
                 for (int j = 0; j < 3; ++j)
                      des_pose_n(i,j) = RefOrFrame(i,j);

            des_pose_n = matrixOrthonormalization(des_pose_n);

            for (int i = 0; i < 3; ++i)
                 for (int j = 0; j < 3; ++j)
                      RefOrFrame(i,j) = des_pose_n(i,j);
          

            /////// INVERSE KINEMATICS ///////
            /*
            //position and orientation compensation between flange and end effector (camera)
            des_pose.p = KDL::Vector(p.pos[0],p.pos[1],p.pos[2])-ee_T_cam.p;
            //des_pose.M = robot.getEEFrame().M*RefOrFrame*KDL::Rotation::RotZ(0.5)*KDL::Rotation::RotZ(1.57)*KDL::Rotation::RotY(-1.57); //offset 30deg              
            des_pose.M = robot.getEEFrame().M*RefOrFrame*KDL::Rotation::RotZ(1.57)*KDL::Rotation::RotY(-1.57);               
            
            //retrieve current values
            qd.data << jnt_pos[0], jnt_pos[1], jnt_pos[2], jnt_pos[3], jnt_pos[4], jnt_pos[5], jnt_pos[6];          
            dqd.data << jnt_vel[0], jnt_vel[1], jnt_vel[2], jnt_vel[3], jnt_vel[4], jnt_vel[5], jnt_vel[6];

            //update desired values
            ddqd = robot.getInvAccKin(qd,dqd,des_cart_acc);
            dqd = robot.getInvVelKin(qd, des_cart_vel);
            qd = robot.getInvKin(qd, des_pose);
            
            /////// JOINT SPACE INVERSE DYNAMICS CONTROL  ///////        
            // Gains
            double Kp = 20, Kd = 10; 
            tau = controller_.idCntr(qd, dqd, ddqd, Kp, Kd);
            */
            
            ///////  OPERATIONAL SPACE INVERSE DYNAMICS CONTROL  ///////        
            
            des_pose.p = KDL::Vector(p.pos[0],p.pos[1],p.pos[2]);
            //des_pose.M = robot.getEEFrame().M*RefOrFrame*KDL::Rotation::RotZ(0.5);
            des_pose.M = robot.getEEFrame().M*RefOrFrame;


            // Gains
            double Kp = 50, Kd = 10; 

            //tau = controller_.idCntr(des_pose.p,e_or,des_cart_vel, des_cart_acc, Kp, Kd, 2*sqrt(Kp), 2*sqrt(Kd)); // neglecting L,assuming wd=0
            tau = controller_.idCntr(des_pose, des_cart_vel, des_cart_acc, Kp, Kd, 2*sqrt(Kp), 2*sqrt(Kd)); 
          
 
            // Alternative solution : operational space inverse dynamics control using euler angles
            //tau = controller_.idCntr_euler(des_pose, des_cart_vel, des_cart_acc, Kp, Kd, 2*sqrt(Kp), 2*sqrt(Kd));
            
            // Set torques
            tau1_msg.data = tau[0];
            tau2_msg.data = tau[1];
            tau3_msg.data = tau[2];
            tau4_msg.data = tau[3];
            tau5_msg.data = tau[4];
            tau6_msg.data = tau[5];
            tau7_msg.data = tau[6];

            // Publish
            joint1_effort_pub.publish(tau1_msg);
            joint2_effort_pub.publish(tau2_msg);
            joint3_effort_pub.publish(tau3_msg);
            joint4_effort_pub.publish(tau4_msg);
            joint5_effort_pub.publish(tau5_msg);
            joint6_effort_pub.publish(tau6_msg);
            joint7_effort_pub.publish(tau7_msg);

            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    if(pauseGazebo.call(pauseSrv))
        ROS_INFO("Simulation paused.");
    else
        ROS_INFO("Failed to pause simulation.");

    return 0;
}