#include <stdio.h>
#include <fstream>
#include <sstream>
#include <string.h>

#include "kdl_ros_control/kdl_robot.h"
#include "kdl_ros_control/kdl_control.h"
#include "kdl_ros_control/kdl_object.h"
#include "kdl_ros_control/kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl_ros_control/utils.h"

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/GetLinkProperties.h"
#include "gazebo_msgs/SetLinkState.h"
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/SetModelConfiguration.h"
#include <std_srvs/Empty.h>

#include <casadi/casadi.hpp>
#include <math.h>
#include <chrono>
#include <eigen_conversions/eigen_kdl.h>
#include "config.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"
#include <shared_control_msgs/GetTorques.h>
#include <shared_control_msgs/requestCalc.h>


#define USE_SHARED_CNTR 1
#define USE_JNT_ID 1
#define DEBUG 0
#define SAVE_DATA 0

// Global variables
std::vector<double> jnt_pos(7,0.0), jnt_vel(7,0.0), obj_pos(6,0.0),  obj_vel(6,0.0);
bool robot_state_available = false, obj_state_available = false;
bool calculation_done = false;
bool torque_ready = false;
double obj_mass;
std_msgs::Float64MultiArray tau_msg;

bool sim_ready = false;
Eigen::Matrix3d obj_inertia = Eigen::Matrix3d::Identity();
const int n_states = 21;
const int n_controls = 7;
const int MPC_HORIZON = 3;
const int n_jnts = 7;
//MPC Fucntions
Eigen::MatrixXd combinedM(const Eigen::MatrixXd &M_m, const Eigen::MatrixXd &M_o, const Eigen::MatrixXd &iJb)//,iJb,M_m);
{
    Eigen::Matrix<double,6,6> M = iJb.transpose()*M_m*iJb + M_o;
    return M;
}
Eigen::MatrixXd combinedC(const Eigen::MatrixXd &M_m,const Eigen::MatrixXd &C_m,const Eigen::MatrixXd &C_o,const Eigen::MatrixXd &iJb,const Eigen::MatrixXd &diJb)
{
    Eigen::Matrix<double,6,6> C = iJb.transpose()*(C_m*iJb + M_m*diJb) + C_o;
    return C;
}
Eigen::VectorXd combinedN(const Eigen::VectorXd &N_m,const Eigen::VectorXd &N_o,const Eigen::MatrixXd &iJb)
{
    Eigen::VectorXd N = iJb.transpose()*N_m + N_o;
    return N;
}
casadi::DM toCasadi(const Eigen::VectorXd &eig_vec)
{
    casadi::DM dm_mat(casadi::Sparsity::dense(eig_vec.size()));  
    std::copy(eig_vec.data(), eig_vec.data() + eig_vec.size(), dm_mat.ptr());
    // std::cout<<"dm:: "<<dm_mat<<std::endl;
    return dm_mat;
}
// void toEigen(const casadi::DM &dm_mat)
// {
//     auto vector_x = static_cast<std::vector<double>>(dm_mat);
//     // int sz = dm_mat.size1();
//     // Eigen::VectorXd eig_vec(sz);
//     // eig_vec << vector_x.data();
//     Eigen::Matrix<float, 2*n_states+n_controls, 1> eig_vec(vector_x.data());
//     // std::cout<<"cas:: "<<eig_vec<<std::endl;
//     // std::cout<<"to eigen "<<vector_x.data()<<std::endl;
//     //return eig_vec;
// }
Eigen::Matrix3d zyx2R(const Eigen::VectorXd &v){ //euler to rotation matrix
    float c_x = cos(v[2]);
    float s_x = sin(v[2]);
    float c_y = cos(v[1]);
    float s_y = sin(v[1]);
    float c_z = cos(v[0]);
    float s_z = sin(v[0]);
  
    Eigen::Matrix3d rot_z, rot_y, rot_x, R;
    rot_z << c_z, -s_z, 0,
            s_z, c_z, 0,
            0, 0, 1;

    rot_y << c_y, 0, s_y,
            0, 1, 0,
            -s_y, 0, c_y;

    rot_x << 1, 0, 0,
            0, c_x, -s_x,
            0, s_x, c_x;

    R << rot_z*rot_y*rot_x;
    return R;       
}
Eigen::Matrix3d zyx2E(const Eigen::VectorXd &v){ //coeff matrix to convert derivatives of euler to ang vel 
    Eigen::Matrix3d E;
    E << 0, sin(v[2])/cos(v[1]), cos(v[2])/cos(v[1]),
         0, cos(v[2]), -sin(v[2]),
         1, sin(v[2])*sin(v[1])/cos(v[1]), cos(v[2])*sin(v[1])/cos(v[1]);
    return E;
}
std::vector<double> ref(const Eigen::MatrixXd &tau_ref, const std::vector<double> &jnt_pos_ref,
                        const std::vector<double> &jnt_vel_ref)
{
    std::vector<double> pVEC_ref;
    for (int i = 0; i < MPC_HORIZON; ++i)
    {   
        for (int j = 0; j < 7; j++)
            pVEC_ref.push_back(tau_ref(i,j));
        for (int j = 0; j < 7; j++)
            pVEC_ref.push_back(jnt_pos_ref[i*n_jnts+j]);
        for (int j = 0; j < 7; j++)
            pVEC_ref.push_back(jnt_vel_ref[i*n_jnts+j]); //ref for jnt vel
        for (int j = 0; j < n_controls; ++j) //ref for tau_dot
            pVEC_ref.push_back(0);
    }
    return pVEC_ref;
}    
void shift(float h, float &t0, Eigen::VectorXd &x0, Eigen::Matrix<double,MPC_HORIZON,n_controls> &u0, const Eigen::MatrixXd &sol_ct_first, const Eigen::MatrixXd &sol_ct_rest, const Eigen::MatrixXd &sol_ct_last, const Eigen::VectorXd &rhs)
{
    Eigen::VectorXd xNew(n_states);
    for (int i = 0; i < rhs.size(); ++i)
    {
      xNew[i] = x0[i] + rhs[i]*h;
    }
    x0 = xNew;
    if (MPC_HORIZON > 1)  
        u0 << sol_ct_rest, sol_ct_last;
    else
        u0 << sol_ct_first;
    t0 = t0 + h;
}
void removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove)
{
    unsigned int numRows = matrix.rows()-1;
    unsigned int numCols = matrix.cols();
    if( rowToRemove < numRows )
        matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) = matrix.block(rowToRemove+1,0,numRows-rowToRemove,numCols);

    matrix.conservativeResize(numRows,numCols);
}
void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove)
{
    unsigned int numRows = matrix.rows();
    unsigned int numCols = matrix.cols()-1;

    if( colToRemove < numCols )
        matrix.block(0,colToRemove,numRows,numCols-colToRemove) = matrix.block(0,colToRemove+1,numRows,numCols-colToRemove);
    matrix.conservativeResize(numRows,numCols);
}
void fromEigenToKDL(const Eigen::Matrix4d &e, KDL::Frame &k)
{
  for (unsigned int i = 0; i < 3; ++i)
    k.p[i] = e(i, 3);
  for (unsigned int i = 0; i < 9; ++i)
    k.M.data[i] = e(i/3, i%3);
}
void fromKDLToEigen(const KDL::Frame &k, Eigen::Matrix4d &e)
{
  // translation
  for (unsigned int i = 0; i < 3; ++i)
    e(i, 3) = k.p[i];

  // rotation matrix
  for (unsigned int i = 0; i < 9; ++i)
    e(i/3, i%3) = k.M.data[i];

  // "identity" row
  e(3,0) = 0.0;
  e(3,1) = 0.0;
  e(3,2) = 0.0;
  e(3,3) = 1.0;
}
std::vector<double> fromKDLToVec(const KDL::JntArray k)
{
    std::vector<double> v;
    for (int i = 0; i < 7; ++i)
        v.push_back(k.data[i]);
    return v;
}
KDL::JntArray fromVecToKDL(std::vector<double> &v)
{
    KDL::JntArray k(7);
    for (int i = 0; i < 7; ++i)
        k.data[i] = v[i];
    return k;
}
// Functions
KDLRobot createRobot(ros::NodeHandle &_n)
{
    KDL::Tree robot_tree;
    std::string robot_desc_string;
    _n.param("robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, robot_tree)){
        ROS_ERROR("Failed to construct kdl tree");
    }
    KDLRobot robot(robot_tree);
    return robot;
}


// Callbacks
void objectStateCallback(const gazebo_msgs::LinkStates & msg)
{
    obj_pos.clear();
    obj_vel.clear();

    obj_pos.push_back(msg.pose[1].position.x);
    obj_pos.push_back(msg.pose[1].position.y);
    obj_pos.push_back(msg.pose[1].position.z);
    obj_pos.push_back(msg.pose[1].orientation.x);
    obj_pos.push_back(msg.pose[1].orientation.y);
    obj_pos.push_back(msg.pose[1].orientation.z);

    obj_vel.push_back(msg.twist[1].linear.x);
    obj_vel.push_back(msg.twist[1].linear.y);
    obj_vel.push_back(msg.twist[1].linear.z);
    obj_vel.push_back(msg.twist[1].angular.x);
    obj_vel.push_back(msg.twist[1].angular.y);
    obj_vel.push_back(msg.twist[1].angular.z);

    obj_state_available = true;
}


void synchCallback( std_msgs::Bool msg ) {

    robot_state_available = (calculation_done && msg.data);
}


void jointStateCallback(const sensor_msgs::JointState & msg)
{

    //robot_state_available = true;
    jnt_pos.clear();
    jnt_vel.clear();
    for (int i = 0; i < msg.position.size(); i++)
    {
        jnt_pos.push_back(msg.position[i]);
        jnt_vel.push_back(msg.velocity[i]);
    }
    sim_ready = true;
    ROS_INFO("read the joint state");
}


bool gettorques(shared_control_msgs::GetTorques::Request &req,
                shared_control_msgs::GetTorques::Response &res)
{
    //res.b = 60.75;

    if (!torque_ready) return false;
    //while ( !torque_ready  ) usleep (0.1*1e6);
    //std::cout << "gettorques" << std::endl;

    res.tj0 = tau_msg.data[0];
    res.tj1 = tau_msg.data[1];
    res.tj2 = tau_msg.data[2];
    res.tj3 = tau_msg.data[3];
    res.tj4 = tau_msg.data[4];
    res.tj5 = tau_msg.data[5];
    res.tj6 = tau_msg.data[6];

    ROS_INFO("sent back the torque");
    return true;
}



bool start_calc(shared_control_msgs::requestCalc::Request &req,
                shared_control_msgs::requestCalc::Response &res)
{

    std::cout << "called" << std::endl;
    robot_state_available = (calculation_done && true);

    return true;
}


// Main
int main(int argc, char **argv)
{
    // Init node
    ros::init(argc, argv, "kdl_ros_control_node");
    ros::NodeHandle n;

    // Rate
    ros::Rate loop_rate(500);
    //modified [mpc parameters]
    int n_total = n_states + n_controls;
    int mpciter = 0;
    float t0 = 0;
    float h = 0.01;
    std::vector<float> t_mpc;
    t_mpc.push_back(t0);
    float epsilon_t = 1e-2;
    //include solver
    std::string path = std::string(nlpN3_man_PATH);
    std::cout << "Library to load: " << path + "/nlpN3_man.so\n";
    /*casadi::FileDeserializer fs("solver.opts");
    casadi::Dict opts = fs.update_dict();*/
    // casadi::Function solv = casadi::nlpsol("solv", "ipopt", path + "/nlpN3_man.so"); //loading solver
    casadi::Function solv= casadi::Function::load(path + "/solver.casadi"); //takes longer time for solver computation
    // casadi::Function solv = casadi::external("nlpN2_man.so"); //cannot be used for ipopt----not codegenerated
    casadi::Function fDYN = casadi::external("f_man"); //loading the dynamics fucntion
    //modified [mpc parameters]
    // Subscribers
    ros::Subscriber joint_state_sub = n.subscribe("/lbr_iiwa/joint_states", 0, jointStateCallback);
    ros::Subscriber synch_sub = n.subscribe("/mpc/sync", 1, synchCallback);
    ros::Subscriber object_state_sub = n.subscribe("/gazebo/link_states", 1, objectStateCallback);
    //ros::ServiceServer service = n.advertiseService("get_torques", gettorques);
    ros::ServiceServer service = n.advertiseService("start_calc", start_calc);

    // Publishers
    // ros::Publisher joint1_effort_pub = n.advertise<std_msgs::Float64>("/lbr_iiwa/lbr_iiwa_joint_1_effort_controller/command", 1);
    ros::Publisher joint2_effort_pub = n.advertise<std_msgs::Float64>("/lbr_iiwa/lbr_iiwa_joint_2_effort_controller/command", 1);
    ros::Publisher joint3_effort_pub = n.advertise<std_msgs::Float64>("/lbr_iiwa/lbr_iiwa_joint_3_effort_controller/command", 1);
    ros::Publisher joint4_effort_pub = n.advertise<std_msgs::Float64>("/lbr_iiwa/lbr_iiwa_joint_4_effort_controller/command", 1);
    ros::Publisher joint5_effort_pub = n.advertise<std_msgs::Float64>("/lbr_iiwa/lbr_iiwa_joint_5_effort_controller/command", 1);
    ros::Publisher joint6_effort_pub = n.advertise<std_msgs::Float64>("/lbr_iiwa/lbr_iiwa_joint_6_effort_controller/command", 1);
    ros::Publisher joint7_effort_pub = n.advertise<std_msgs::Float64>("/lbr_iiwa/lbr_iiwa_joint_7_effort_controller/command", 1);
    //
    ros::Publisher joint_tor_pub = n.advertise<std_msgs::Float64MultiArray>("/torque_joints", 1);
    //
    ros::ServiceClient obj_set_state_srv = n.serviceClient<gazebo_msgs::SetLinkState>("/gazebo/set_link_state");
    ros::ServiceClient des_pose_set_state_srv = n.serviceClient<gazebo_msgs::SetLinkState>("/gazebo/set_link_state");
    ros::ServiceClient obj_get_dyn_srv = n.serviceClient<gazebo_msgs::GetLinkProperties>("/gazebo/get_link_properties");
    ros::ServiceClient robot_set_state_srv = n.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");
    ros::ServiceClient resetGazeboSimulation = n.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation");
    ros::ServiceClient pauseGazebo = n.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    ros::ServiceClient unpauseGazebo = n.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");

    // Messages
    //std_msgs::Float64 tau1_msg, tau2_msg, tau3_msg, tau4_msg, tau5_msg, tau6_msg, tau7_msg;
    tau_msg.data.resize(7);
    // Services
//    std_srvs::Empty reset_simulation_srv;
//    if(resetGazeboSimulation.call(reset_simulation_srv))
//        ROS_INFO("Reset simulation.");
//    else
//        ROS_INFO("Failed to reset simulation.");

    gazebo_msgs::SetLinkState obj_init_state;
    obj_init_state.request.link_state.link_name = "lbr_iiwa::cuboid_link";
    obj_init_state.request.link_state.twist.linear.x = 0.0;
    obj_init_state.request.link_state.twist.linear.y = 0.0;
    obj_init_state.request.link_state.twist.linear.z = 0.0;
    obj_init_state.request.link_state.twist.angular.x = 0.0;
    obj_init_state.request.link_state.twist.angular.y = 0.0;
    obj_init_state.request.link_state.twist.angular.z = 0.0;
    obj_init_state.request.link_state.reference_frame = "world";
    obj_init_state.request.link_state.pose.position.x = 0.594973; // 0.564973;
    obj_init_state.request.link_state.pose.position.y = -0.312973; // 0.372973
    obj_init_state.request.link_state.pose.position.z = 0.522153;
    obj_init_state.request.link_state.pose.orientation.x = 0.0;
    obj_init_state.request.link_state.pose.orientation.y = 0.0;
    obj_init_state.request.link_state.pose.orientation.z = 0.0;
    if(obj_set_state_srv.call(obj_init_state))
        ROS_INFO("Object state set.");
    else
        ROS_INFO("Failed to set object state.");

    /*gazebo_msgs::SetLinkState des_pose_init_state;
    des_pose_init_state.request.link_state.link_name = "lbr_iiwa::cuboid_link";
    des_pose_init_state.request.link_state.reference_frame = "world";
    des_pose_init_state.request.link_state.pose.position.x = 0.594973; // 0.564973;
    des_pose_init_state.request.link_state.pose.position.y = -0.312973; // 0.372973
    des_pose_init_state.request.link_state.pose.position.z = 0.522153;
    des_pose_init_state.request.link_state.pose.orientation.x = 0.0;
    des_pose_init_state.request.link_state.pose.orientation.y = 0.0;
    des_pose_init_state.request.link_state.pose.orientation.z = 0.0;
    if(des_pose_set_state_srv.call(des_pose_init_state))
        ROS_INFO("Desired pose state set.");
    else
        ROS_INFO("Failed to set desired pose state.");
    */
    
    gazebo_msgs::SetModelConfiguration robot_init_config;
    robot_init_config.request.model_name = "lbr_iiwa";
    robot_init_config.request.urdf_param_name = "robot_description";
    robot_init_config.request.joint_names.push_back("lbr_iiwa_joint_1");
    robot_init_config.request.joint_names.push_back("lbr_iiwa_joint_2");
    robot_init_config.request.joint_names.push_back("lbr_iiwa_joint_3");
    robot_init_config.request.joint_names.push_back("lbr_iiwa_joint_4");
    robot_init_config.request.joint_names.push_back("lbr_iiwa_joint_5");
    robot_init_config.request.joint_names.push_back("lbr_iiwa_joint_6");
    robot_init_config.request.joint_names.push_back("lbr_iiwa_joint_7");
    robot_init_config.request.joint_positions.push_back(0.107264);
    robot_init_config.request.joint_positions.push_back(1.5257);
    robot_init_config.request.joint_positions.push_back(-1.60002);
    robot_init_config.request.joint_positions.push_back(-1.2156);
    robot_init_config.request.joint_positions.push_back(1.53864);
    robot_init_config.request.joint_positions.push_back(-1.52774);
    robot_init_config.request.joint_positions.push_back(1.10969);
    if(robot_set_state_srv.call(robot_init_config))
        ROS_INFO("Robot state set.");
    else
        ROS_INFO("Failed to set robot state.");

    gazebo_msgs::GetLinkProperties obj_dyn_prop;
    obj_dyn_prop.request.link_name = "lbr_iiwa::cuboid_link";
    if(obj_get_dyn_srv.call(obj_dyn_prop))
    {
        ROS_INFO("Object dynamic properties retrieved.");
        obj_mass = obj_dyn_prop.response.mass;
        obj_inertia << obj_dyn_prop.response.ixx,
                obj_dyn_prop.response.ixy,
                obj_dyn_prop.response.ixz,
                obj_dyn_prop.response.ixy,
                obj_dyn_prop.response.iyy,
                obj_dyn_prop.response.iyz,
                obj_dyn_prop.response.ixz,
                obj_dyn_prop.response.iyz,
                obj_dyn_prop.response.izz;
        std::cout << "Object mass: " << obj_mass << std::endl;
        std::cout << "Object inertia: " << std::endl << obj_inertia << std::endl;
    }
    else
        ROS_INFO("Failed to get object dynamic properties.");

    std_srvs::Empty pauseSrv;


    // Wait for robot and object state
    // unpauseGazebo.call(pauseSrv);
    calculation_done = true;
    while(!(sim_ready && robot_state_available) ) // || !obj_state_available)
    {
        ROS_INFO_STREAM_ONCE("Robot/object state not available yet.");
        ROS_INFO_STREAM_ONCE("Please start gazebo simulation.");
        ros::spinOnce();

    }
    // robot_state_available = false;

    //pauseGazebo.call(pauseSrv);

    // Create robot
    KDLRobot robot = createRobot(n);
    /*jnt_pos[0] = 0.107264; jnt_pos[1] = 1.5257; jnt_pos[2] = -1.60002; jnt_pos[3] = -1.2156;
    jnt_pos[4] = 1.53864; jnt_pos[5] = -1.52774; jnt_pos[6] = 1.10969;*/
    robot.update(jnt_pos, jnt_vel);

    // Add end-effector
    KDL::Frame f_T_ee = KDL::Frame::Identity(); f_T_ee.p[2] = 0.016; // plate height
    robot.addEE(f_T_ee);

    // Create object
    double frictionCoeff = 0.5;
    std::vector<KDL::Frame> contacts(4);
    contacts.at(0).p = KDL::Vector(-0.02, 0.02,-0.02);
    contacts.at(1).p = KDL::Vector(0.02, 0.02,-0.02);
    contacts.at(2).p = KDL::Vector(0.02,-0.02,-0.02);
    contacts.at(3).p = KDL::Vector(-0.02,-0.02,-0.02);
    KDLObject obj(obj_mass, obj_inertia, contacts, frictionCoeff);
    std::cout << toEigen(obj.getFrame().M) << std::endl;
    KDL::Wrench Fb(obj.getFrame().M.Inverse()*KDL::Vector(0,0,9.81*obj_mass), KDL::Vector(0,0,0));
    obj.computeContactForces(Fb);

    obj.setFrame(toKDL(obj_pos));

    // Add object
    // robot.addObj(obj);

    KDL::Frame ee_F_obj_init = robot.getEEFrame().Inverse()*obj.getFrame();
    std::cout << "ee_F_obj_init: " << std::endl << ee_F_obj_init << std::endl;
    KDL::Frame s_F_obj_const = KDL::Frame::Identity(); s_F_obj_const.p = obj.getFrame().p; // plate height
    std::cout << "s_F_obj_const: " << std::endl << s_F_obj_const << std::endl;
    std::cout << "s_F_ee_initial: " << std::endl << robot.getEEFrame() << std::endl;
    KDL::JntArray jnt_ik_kdl, jnt_init_kdl; jnt_ik_kdl.resize(7);
    KDL::JntArray jnt_vel_ik_kdl; jnt_vel_ik_kdl.resize(n_jnts);
    jnt_init_kdl = fromVecToKDL(jnt_pos);
    std::cout << "Jnt pos before " << jnt_pos << std::endl;
    jnt_ik_kdl = robot.getInvKin(jnt_init_kdl, s_F_obj_const);
    jnt_pos = fromKDLToVec(jnt_ik_kdl);
    robot.update(jnt_pos, jnt_vel);
    std::cout << "Jnt pos after " << jnt_pos << std::endl;
    jnt_init_kdl = fromVecToKDL(jnt_pos);
    KDL::Frame s_F_ee_ik;
    // Joints
    KDL::JntArray qd(robot.getNrJnts()),dqd(robot.getNrJnts()),ddqd(robot.getNrJnts());

    // Torques
    Eigen::VectorXd tau;
    tau.resize(robot.getNrJnts());

    // Update robot
    robot.update(jnt_pos, jnt_vel);

    // Initial object frame
    KDL::Frame init_cart_pose = obj.getFrame();
    Eigen::Vector3d init_position(init_cart_pose.p.data);

    // Final object frame
    Eigen::Vector3d end_position;
    end_position << init_cart_pose.p.x(), -init_cart_pose.p.y(), init_cart_pose.p.z();

    // Plan trajectory
    double traj_duration = 2, acc_duration = 1, t = 0.0, init_time_slot = 0.0;
    KDLPlanner planner(traj_duration, acc_duration, init_position, end_position);
    trajectory_point p = planner.compute_trajectory(t);
    //***************modified [mpc parameters]*******************
    std::vector<trajectory_point> p_future(MPC_HORIZON); 
    std::vector<double> jnt_pos_ref(MPC_HORIZON*n_jnts,0.0);
    std::vector<double> jnt_vel_ref(MPC_HORIZON*n_jnts,0.0);
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(n_states);
    Eigen::Matrix<double,7,7> M_m; //to strore the combined inertia matrix
    Eigen::VectorXd C_m; //to strore the combined coriolis matrix
    Eigen::VectorXd N_m; //to strore the combined gravity matrix
    Eigen::Matrix<double,7,1> tau_mpc;
    for (int i = 0; i < 7; ++i)
        jnt_pos_ref[i] = jnt_pos[i];

    N_m = robot.getGravity();
    for (int i = 0; i < 7; ++i)
        x0(i) = N_m(i);

    for (int i = 7; i < 14; ++i)
        x0(i) = jnt_pos_ref[i-7];

    std::cout<<"#############x0#################"<<x0<<std::endl;
    // Eigen::VectorXd xx = Eigen::VectorXd::Zero(n_states);
    // xx << x0;
    std::vector<double> xx(x0.data(), x0.data() + x0.rows() * x0.cols());
    Eigen::Matrix<double,100,n_states> xx_out = Eigen::Matrix<double,100,n_states>::Zero();
    xx_out.row(mpciter) << x0.transpose();
    Eigen::VectorXd x_prev = Eigen::VectorXd::Zero(n_states);
    Eigen::Matrix<double,MPC_HORIZON,n_controls> u0 = Eigen::Matrix<double,MPC_HORIZON,n_controls>::Zero();
    Eigen::Matrix<double,MPC_HORIZON+1,n_states> X0 = Eigen::Matrix<double,MPC_HORIZON+1,n_states>::Zero();
    for (int i = 0; i < MPC_HORIZON+1; ++i)
    {
        for (int j = 0; j < n_states; ++j)
            X0(i,j) = x0[j];
    }
    std::cout<<"#############X0#################"<<X0<<std::endl;
    Eigen::VectorXd args_x0 = Eigen::VectorXd::Zero(n_states+MPC_HORIZON*n_total);

    // std::cout<<"#############M_combined#################"<<M_<<std::endl;
    // std::cout<<"#############C_combined#################"<<C_<<std::endl;
    // std::cout<<"#############N_combined#################"<<N_<<std::endl;

    std::vector<double> pVEC_ref; //std vector because we want to push x0 u0 alternately
    Eigen::VectorXd pVEC_ref_; //to keep converted std vector to eigen vector
    Eigen::VectorXd pVEC(63+4+n_states+MPC_HORIZON*n_total); //to keep the entire p argument
    // bounds for states and controls
    Eigen::VectorXd st_lbx = Eigen::VectorXd::Zero(n_states);
    st_lbx << -500*Eigen::VectorXd::Ones(7), -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96, -100*Eigen::VectorXd::Ones(7);
    Eigen::VectorXd st_ubx = Eigen::VectorXd::Zero(n_states);
    st_ubx << 500*Eigen::VectorXd::Ones(7), 2.96,2.09,2.96,2.09,2.96,2.09,2.96, 100*Eigen::VectorXd::Ones(7);
    Eigen::VectorXd ct_lbx = -1000*Eigen::VectorXd::Ones(n_controls);
    Eigen::VectorXd ct_ubx = 1000*Eigen::VectorXd::Ones(n_controls);
    Eigen::VectorXd lbx(n_states+MPC_HORIZON*n_total);
    Eigen::VectorXd ubx(n_states+MPC_HORIZON*n_total);
    for (int i = 0; i < MPC_HORIZON+1; ++i)
    {
        for (int j = 0; j < n_states; ++j)
        {
            lbx(i*n_states+j) = st_lbx(j);
            ubx(i*n_states+j) = st_ubx(j);
        }
    }
    for (int i = 0; i < MPC_HORIZON; ++i)
    {
        for (int j = 0; j < n_controls; ++j)
        {
            lbx((MPC_HORIZON+1)*n_states+i*n_controls+j) = ct_lbx(j);
            ubx((MPC_HORIZON+1)*n_states+i*n_controls+j) = ct_ubx(j);
        }
    }
    // bounds for constaints
    Eigen::VectorXd st_lbg = -1e-20*Eigen::VectorXd::Ones(n_states);
    Eigen::VectorXd st_ubg = 1e-20*Eigen::VectorXd::Ones(n_states);
    Eigen::VectorXd lbg((MPC_HORIZON+1)*n_states);
    Eigen::VectorXd ubg((MPC_HORIZON+1)*n_states);
    for (int i = 0; i < MPC_HORIZON+1; ++i)
    {
        for (int j = 0; j < n_states; ++j)
        {
            lbg(i*n_states+j) = st_lbg(j);
            ubg(i*n_states+j) = st_ubg(j);
        }
    }
    Eigen::VectorXd lam_x0 = Eigen::VectorXd::Zero(n_states+MPC_HORIZON*n_total);
    Eigen::VectorXd lam_g0 = Eigen::VectorXd::Zero((MPC_HORIZON+1)*n_states);

    casadi::DM args_x0_cas; //to keep converted quantities from eigen to casadi
    casadi::DM p_cas;
    casadi::DM lbx_cas = toCasadi(lbx);
    casadi::DM ubx_cas = toCasadi(ubx);
    casadi::DM lbg_cas = toCasadi(lbg);
    casadi::DM ubg_cas = toCasadi(ubg);
    casadi::DM lam_x0_cas = toCasadi(lam_x0);
    casadi::DM lam_g0_cas = toCasadi(lam_g0);

    std::vector<casadi::DM> arg;
    std::vector<casadi::DM> res;
    std::chrono::duration<double> elapsed_seconds; 
    casadi::DM sol_cas; //to keep solution from the solver
    Eigen::MatrixXd sol_ct_all(MPC_HORIZON*n_controls,1);
    Eigen::MatrixXd sol_ct_colWise;
    Eigen::Matrix<double, 1, n_controls> sol_ct_first;
    Eigen::Matrix<double, 1, n_controls> sol_ct_last;
    //should account for MPC_HORIZON=1
    // if (MPC_HORIZON > 1)
        Eigen::Matrix<double, (MPC_HORIZON-1), n_controls> sol_ct_rest;
    // else 
    //     Eigen::MatrixXd sol_ct_rest = Eigen;
    t_mpc[mpciter+1] = t0;
    casadi::DM sol_ct_first_cas;
    casadi::DM x0_cas;
    casadi::DM M_m_cas, C_m_cas, N_m_cas;
    std::vector<casadi::DM> argF;
    std::vector<casadi::DM> f_value; //call the dynamic fucntion
    casadi::DM rhs_cas;
    double t_tau_now = 0, t_tau_prev = 0; //time keeping for tau calculation
    double t_sh_now = 0, t_sh_prev = 0; //time keeping for shift function
    Eigen::Matrix<double, MPC_HORIZON, n_jnts> tau_ref = Eigen::Matrix<double, MPC_HORIZON, n_jnts>::Zero();  
    Eigen::Matrix<double, MPC_HORIZON, n_jnts> q_ref = Eigen::Matrix<double, MPC_HORIZON, n_jnts>::Zero();
    //***************modified [mpc parameters]*******************
    //momentumEstimator est(5.0, robot.getNrJnts());

    // Create controller
    KDLController controller_(robot, obj);

    // Gains
    double Kp = 50, Kd = sqrt(Kp);

    gazebo_msgs::SetLinkState obj_current_state = obj_init_state;
    KDL::Vector obj_init_pos(obj_init_state.request.link_state.pose.position.x,
                             obj_init_state.request.link_state.pose.position.y,
                             obj_init_state.request.link_state.pose.position.z);

#if SAVE_DATA
    std::string robot_file_name = "robot_" + std::to_string(i*spatial_step) + "_" + std::to_string(j*spatial_step) + ".txt";
    std::string path_file_name = "path_" + std::to_string(i*spatial_step) + "_" + std::to_string(j*spatial_step) + ".txt";
    std::string obj_file_name = "obj_" + std::to_string(i*spatial_step) + "_" + std::to_string(j*spatial_step) + ".txt";

    std::ofstream robot_file(robot_file_name);
    std::ofstream path_file(path_file_name);
    std::ofstream obj_file(obj_file_name);
#endif
    // Reset robot/object state
    // unpauseGazebo.call(pauseSrv);

//    robot_state_available = false;
//    while(!robot_state_available)
//        ros::spinOnce();

    // Retrieve initial simulation time
    ros::Time begin = ros::Time::now();
    ROS_INFO_STREAM_ONCE("Starting control loop ...");
    t_tau_prev = (ros::Time::now()-begin).toSec();
    t_sh_prev = (ros::Time::now()-begin).toSec();
    // auto t_tau_prev = std::chrono::steady_clock::now();
    double t_tau_diff = 0; //std::chrono::duration<double> t_tau_diff; 
    std::vector<double> GAINS(4,0.0);
    GAINS[0] = 1e-3; GAINS[1] = 100; GAINS[2] = 10; GAINS[3] = 1e-3; //(Q_tau, Q_q, Q_dq, R)
    std::vector<double> jnt_ik(7,0.0);
    while ((ros::Time::now()-begin).toSec() < 2*traj_duration + init_time_slot)
    {
        //unpauseGazebo.call(pauseSrv);
        //if (robot_state_available)
        //{
        while ( !robot_state_available ) {
            usleep(0.1*1e6);
            ros::spinOnce();
        }

        calculation_done = false;

        robot_state_available = false;

        std::cout << "robot_state_available" << std::endl;
        if (sim_ready)
        {
        std::cout << "sim ready" << std::endl;
        // Update robot
        robot.update(jnt_pos, jnt_vel);
        std::cout<<"Joint pos: "<<jnt_pos<<std::endl;
        std::cout<<"Joint vel: "<<jnt_vel<<std::endl;
        // Update time
        t = (ros::Time::now()-begin).toSec();
        std::cout << "time: " << t << "\n";
        //modified [mpc parameters]
        pVEC_ref.clear(); //empty the std vector to push back again
        M_m = robot.getJsim();
        C_m = robot.getCoriolis();
        N_m = robot.getGravity();
        //modified [mpc parameters]
        // Extract desired pose
        KDL::Frame des_pose = KDL::Frame::Identity();
        KDL::Twist des_cart_vel = KDL::Twist::Zero(), des_cart_acc = KDL::Twist::Zero();
        if (t < init_time_slot) // wait a second
        {
            std::cout << "here!! " << "\n";
            p = planner.compute_trajectory(0.0);
        }
        else if(t >= init_time_slot && t <= traj_duration + init_time_slot)
        {
            p = planner.compute_trajectory(t-init_time_slot);
            std::cout << "ok1" << std::endl;
            //modified [mpc parameters]
            for (int i = 0; i < MPC_HORIZON; ++i){
                p_future[i] = planner.compute_trajectory(t+i*epsilon_t-init_time_slot);
                std::cout<<"p_future: "<<p_future[i].pos<<" "<<p_future[i].vel<<"\n";
                Eigen::Matrix<double, 6, 1> vel_ref = Eigen::Matrix<double, 6, 1>::Zero();
                Eigen::Matrix<double, 6, 1> acc_ref = Eigen::Matrix<double, 6, 1>::Zero();
                vel_ref << p_future[i].vel[0], p_future[i].vel[1], p_future[i].vel[2], 0, 0, 0;
                acc_ref << p_future[i].acc[0], p_future[i].acc[1], p_future[i].acc[2], 0, 0, 0; 
                tau_ref.block(i,0, 1, 7) = N_m.transpose(); //not exactly accurate ref as state of robot + obj is not updated  
                
                s_F_obj_const.p[0] = p_future[i].pos[0]; s_F_obj_const.p[1] = p_future[i].pos[1]; 
                s_F_obj_const.p[2] = p_future[i].pos[2];
                /*s_F_ee_ik = s_F_obj_const*ee_F_obj_init.Inverse();
                std::cout<<"sFeeIK: " << s_F_ee_ik << "\n";*/
                jnt_ik_kdl = robot.getInvKin(jnt_init_kdl, s_F_obj_const);
                jnt_vel_ik_kdl.data = (jnt_ik_kdl.data - jnt_init_kdl.data)/epsilon_t;
                std::cout<<"jnt pos ik: " << jnt_ik_kdl.data << "\n";
                std::cout<<"jnt vel ik: " << jnt_vel_ik_kdl.data << "\n";
                // q_ref.block(i,0, 1, 7) = jnt_ik_kdl.data;
                for (int j = 0; j < n_jnts; ++j) {
                    jnt_pos_ref[i*n_jnts+j] = jnt_ik_kdl.data(j);
                    jnt_vel_ref[i*n_jnts+j] = jnt_vel_ik_kdl.data(j);
                }
                // std::cout<<"jnt vel ik: " << jnt_vel_ik_kdl.data << "\n";
                jnt_init_kdl = jnt_ik_kdl;
            }
            // std::vector<double> jnt_vel_ik_kdl(7,0.0);
            // robot.update(fromKDLToVec(jnt_ik_kdl),jnt_vel_ik_kdl);     
            pVEC_ref = ref(tau_ref, jnt_pos_ref, jnt_vel_ref);
            pVEC_ref_ = Eigen::Map<Eigen::VectorXd>(pVEC_ref.data(), pVEC_ref.size());
            // Eigen matrices are stored in column major order by default
            pVEC << (Eigen::Map<Eigen::VectorXd>(M_m.data(), M_m.cols()*M_m.rows())), 
                    C_m,
                    N_m,
                    (Eigen::Map<Eigen::VectorXd>(GAINS.data(), GAINS.size())),
                    x0,
                    pVEC_ref_;
            // std::cout<<"pVEC updated "<<pVEC<<std::endl;
            std::cout << "ok2" << std::endl;
            for (int i = 0; i < MPC_HORIZON+1; ++i)
            {
                for (int j = 0; j < n_states; ++j)
                    args_x0(i*n_states+j) = X0(i,j);
            }
            std::cout << "ok3" << std::endl;
            for (int i = 0; i < MPC_HORIZON; ++i)
            {
                for (int j = 0; j < n_controls; ++j)
                    args_x0(n_states*(MPC_HORIZON+1)+i*n_controls+j) = u0(i,j); 
            }
            args_x0_cas = toCasadi(args_x0); //this is extended x
            p_cas = toCasadi(pVEC);
            std::cout << "ok4" << std::endl;
            arg = {args_x0_cas,p_cas,lbx_cas,ubx_cas,lbg_cas,ubg_cas,lam_x0_cas,lam_g0_cas};
            auto start = std::chrono::steady_clock::now();
            std::cout << "ok5" << std::endl;
            if (mpciter == 22) {
            std::cout<<"#############args.x0#################"<<args_x0_cas<<std::endl;
            std::cout<<"#############args.p#################"<<p_cas<<std::endl;
            std::cout<<"#############args.lbx#################"<<lbx_cas<<std::endl;
            std::cout<<"#############args.ubx#################"<<ubx_cas<<std::endl;
            std::cout<<"#############args.lbg#################"<<lbg_cas<<std::endl;
            std::cout<<"#############args.ubg#################"<<ubg_cas<<std::endl;
            }
            res = solv(arg); //call the solver
            auto end = std::chrono::steady_clock::now();
            elapsed_seconds = end-start; 
            std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";
            std::cout << "Objective at solution = " << res.at(1) << std::endl;
            std::cout << "Solution at 0 = " << res.at(0) << std::endl;
            sol_cas = res.at(0); //solution from the solver
            // convert solution for casadi to eigen
            auto sol_vec = static_cast<std::vector<double>>(sol_cas);
            Eigen::Matrix<double, (MPC_HORIZON+1)*n_states+MPC_HORIZON*n_controls, 1> sol_eig(sol_vec.data());
            // std::cout<<"sol entire: "<<sol_eig<<std::endl;
            sol_ct_all << sol_eig.block((MPC_HORIZON+1)*n_states,0,MPC_HORIZON*n_controls,1);
            Eigen::Map<Eigen::MatrixXd> sol_ct_temp(sol_ct_all.data(), n_controls, MPC_HORIZON); 
            sol_ct_colWise = sol_ct_temp.transpose();
            sol_ct_first = sol_ct_colWise.block(0,0,1,n_controls);//(1,Eigen::all);
            sol_ct_last = sol_ct_colWise.block(MPC_HORIZON-1, 0, 1, n_controls);
            //should account for MPC_HORIZON=1
            // if (MPC_HORIZON > 1)
            sol_ct_rest = sol_ct_colWise.block(1, 0, (MPC_HORIZON-1), n_controls);
            // else 
            //     Eigen::MatrixXd sol_ct_rest = Eigen;
            t_mpc[mpciter+1] = t0;
            sol_ct_first_cas = toCasadi(sol_ct_first);
            x0_cas = toCasadi(x0);
            // C_m_cas = toCasadi((Eigen::Map<Eigen::VectorXd>(C_m.data(), C_m.cols()*C_m.rows())));
            M_m_cas = toCasadi((Eigen::Map<Eigen::VectorXd>(M_m.data(), M_m.cols()*M_m.rows())));
            C_m_cas = toCasadi(C_m); //here we are taking 7x1 vector
            N_m_cas = toCasadi(N_m);
            std::cout<<"#############sol_ct_first_cas#################"<<sol_ct_first_cas<<std::endl;
            argF = {x0_cas,sol_ct_first_cas,M_m_cas,C_m_cas,N_m_cas};
            f_value = fDYN(argF); //call the dynamic fucntion
            std::cout << "ok6" << std::endl;
            rhs_cas = f_value.at(0);
            // convert from casadi to eigen
            auto rhs_vec = static_cast<std::vector<double>>(rhs_cas);
            Eigen::Matrix<double, n_states, 1> rhs_eig(rhs_vec.data());
            std::cout<<"rhs: "<<rhs_eig<<std::endl; 
            // std::cout<<"u0 before shift"<<u0<<std::endl;
            t_sh_now = (ros::Time::now()-begin).toSec();
            shift(epsilon_t, t0, x0, u0, sol_ct_first, sol_ct_rest, sol_ct_last, rhs_eig);
            /***use x0 as read from simulator***/
            std::cout << "shift time diff: " << (t_sh_now-t_sh_prev) << "\n";            
            std::cout << "shift time now: " << (t_sh_now) << "\n";            
            std::cout << "shift time prev: " << (t_sh_prev) << "\n";            
            t_sh_prev = t_sh_now; 
            // std::cout<<"x0 after shift"<<x0<<std::endl;
            // std::cout<<"u0 after shift"<<u0<<std::endl; 
            for (int i = 0; i < n_states; ++i)
                xx.push_back(x0[i]);
            xx_out.row(mpciter+1) << x0.transpose();
            // std::cout<<"xx final"<<xx<<std::endl;
            Eigen::Map<Eigen::MatrixXd> X0_temp(sol_eig.block(0,0,(MPC_HORIZON+1)*n_states,1).data(), n_states, MPC_HORIZON+1);
            X0 = X0_temp.transpose();                   
            // shift trajectory to initialize the next step
            X0 << X0.block(1,0,MPC_HORIZON,n_states), X0.block(MPC_HORIZON-1,0,1,n_states);            
            
            std::cout<<"mpciter: "<<mpciter<<"\n";
            mpciter += 1;
            
            x_prev = xx_out.row(mpciter).transpose();
            // tau_mpc = Jb.transpose()*(M_*(x0.block(6,0,6,1) - x_prev.block(6,0,6,1))/h + C_*x0.block(6,0,6,1) + N_);
            t_tau_now = (ros::Time::now()-begin).toSec();
            // auto t_tau_now = std::chrono::steady_clock::now();
            t_tau_diff = t_tau_now-t_tau_prev;
            // std::cout << "vel diff: " << (x0.block(6,0,6,1) - obj_t_Eigen) << "\n";

            // if((t_tau_now-t_tau_prev) < 1e-6)
                tau_mpc = x0.block(0,0,7,1); 
            /*else{    
                tau_mpc = Jb_t*(M_*(x0.block(6,0,6,1) - obj_t_Eigen)/t_tau_diff + C_*x0.block(6,0,6,1) + N_);
                std::cout << "Term1: " << M_*(x0.block(6,0,6,1) - obj_t_Eigen)/t_tau_diff << std::endl;
                std::cout << "Term1a: " << M_ << std::endl;
                std::cout << "Term1b: " << (x0.block(6,0,6,1) - obj_t_Eigen)/t_tau_diff << std::endl;
                std::cout << "Term2: " << C_*x0.block(6,0,6,1) << std::endl;
                std::cout << "Term3: " << N_ << std::endl;
                std::cout << "Term4: " << Jb_t << std::endl;
            }*/
            t_tau_prev = t_tau_now;
            /*for(int i = 0; i < 6; ++i){
              obj_t_update[i] = x0[6+i];
              std::cout << "setting: " << x0[6+i] <<"\n";
            }
            obj.setBodyVelocity(obj_t_update);*/
            //std::cout<<"tau mpc: "<<tau_mpc<<"\n";
            // std::cout<<"Jbt: "<<Jb_t<<"\n";
            // std::cout<<"M_: "<<M_<<"\n";
            // std::cout<<"C_: "<<C_<<"\n";
            // std::cout<<"N_: "<<N_<<"\n";
            //std::cout<<"ddx: "<<(x0.block(6,0,6,1) - obj_t_Eigen)/h<<"\n";
            //std::cout<<"dx: "<<x0.block(6,0,6,1)<<"\n";
            // return 0;
            //modified [mpc parameters]
//                double x = p.pos.x()+ 0.1*sin(2*t);
//                p.pos.x() = x;
            des_cart_vel = KDL::Twist(KDL::Vector(p.vel[0], p.vel[1], p.vel[2]),KDL::Vector::Zero());
            des_cart_acc = KDL::Twist(KDL::Vector(p.acc[0], p.acc[1], p.acc[2]),KDL::Vector::Zero());


            calculation_done = true;
            


            tau_msg.data[0] = tau_mpc[0];
            tau_msg.data[1] = tau_mpc[1];
            tau_msg.data[2] = tau_mpc[2];
            tau_msg.data[3] = tau_mpc[3];
            tau_msg.data[4] = tau_mpc[4];
            tau_msg.data[5] = tau_mpc[5];
            tau_msg.data[6] = tau_mpc[6];

            joint_tor_pub.publish(tau_msg);
            std::cout << "PUBBBLICO TAU!!!!!" << std::endl;
        }
        else
        {
            // std::cout << "time: " << t << "\n";
            ROS_INFO_STREAM_ONCE("trajectory terminated");
        }

        des_pose.p = KDL::Vector(p.pos[0],p.pos[1],p.pos[2]);

#if USE_SHARED_CNTR
        // tau = controller_.sharedCntr(des_pose, des_cart_vel, des_cart_acc, Kp, Kd);
        tau = tau_mpc;
        std::cout << "tau: " << tau.transpose() << std::endl;
#endif //USE_SHARED_CNTR

#if USE_JNT_ID
//            // inverse kinematics
//            des_pose=des_pose*j_T_ee.Inverse();
//            qd.data << jnt_pos[0], jnt_pos[1], jnt_pos[2], jnt_pos[3], jnt_pos[4], jnt_pos[5], jnt_pos[6];
//            robot.getInverseKinematics(des_pose,des_cart_vel,des_cart_acc,qd,dqd,ddqd);

//            // joint space inverse dynamics control
//            tau = controller_.idCntr(qd, dqd, ddqd, Kp, Kd);
#else
        double Kp = 1000;
        double Ko = 1000;
        // Cartesian space inverse dynamics control
        tau = controller_.idCntr(des_pose, des_cart_vel, des_cart_acc,
                                 Kp, Ko, 2*sqrt(Kp), 2*sqrt(Ko));
#endif

#if SAVE_DATA
        double ad,bd,cd, a,b,c;
        KDL::Frame T = robot.getCartesianPose();
        KDL::Twist V = robot.getCartesianVelocity();
        Eigen::VectorXd jnt_pos = robot.getJntValues(), jnt_vel = robot.getJntVelocities();

        des_pose.M.GetEulerZYX(ad,bd,cd);
        T.M.GetEulerZYX(a,b,c);

        path_file << des_pose.p.x() << " " << des_pose.p.y() << " " << des_pose.p.z() << " "
                  << ad << " " << bd << " " << cd << " "

                  << des_cart_vel.vel.x() << " " << des_cart_vel.vel.y() << " " << des_cart_vel.vel.z() << " "
                  << des_cart_vel.rot.x() << " " << des_cart_vel.rot.y() << " " << des_cart_vel.rot.z() << " "

                  << des_cart_acc.vel.x() << " " << des_cart_acc.vel.y() << " " << des_cart_acc.vel.z() << " "
                  << des_cart_acc.rot.x() << " " << des_cart_acc.rot.y() << " " << des_cart_acc.rot.z() << "\n";

        robot_file << T.p.x() << " " << T.p.y() << " " << T.p.z() << " "
                   << a << " " << b << " " << c << " "

                   << V.vel.x() << " " << V.vel.y() << " " << V.vel.z() << " "
                   << V.rot.x() << " " << V.rot.y() << " " << V.rot.z() << " "

                   << tau[0] << " " << tau[1] << " " << tau[2] << " "
                   << tau[3] << " " << tau[4] << " " << tau[5] << " "
                   << tau[6] << " "

                   << qd.data[0] << " " << qd.data[1] << " " << qd.data[2] << " "
                   << qd.data[3] << " " << qd.data[4] << " " << qd.data[5] << " "
                   << qd.data[6] << " "

                   << dqd.data[0] << " " << dqd.data[1] << " " << dqd.data[2] << " "
                   << dqd.data[3] << " " << dqd.data[4] << " " << dqd.data[5] << " "
                   << dqd.data[6] << " "

                   << ddqd.data[0] << " " << ddqd.data[1] << " " << ddqd.data[2] << " "
                   << ddqd.data[3] << " " << ddqd.data[4] << " " << ddqd.data[5] << " "
                   << ddqd.data[6] << " "

                   << jnt_pos[0] << " " << jnt_pos[1] << " " << jnt_pos[2] << " "
                   << jnt_pos[3] << " " << jnt_pos[4] << " " << jnt_pos[5] << " "
                   << jnt_pos[6] << " "

                   << jnt_vel[0] << " " << jnt_vel[1] << " " << jnt_vel[2] << " "
                   << jnt_vel[3] << " " << jnt_vel[4] << " " << jnt_vel[5] << " "
                   << jnt_vel[6] << "\n";

        obj_file << obj_pos[0] << " " << obj_pos[1] << " " << obj_pos[2] << " "
                               << obj_pos[3] << " " << obj_pos[4] << " " << obj_pos[5] << " "
                               << obj_vel[0] << " " << obj_vel[1] << " " << obj_vel[2] << " "
                               << obj_vel[3] << " " << obj_vel[4] << " " << obj_vel[5] << "\n";

        path_file.flush();
        robot_file.flush();
        obj_file.flush();
#endif
        // Set torques
        /*
        tau1_msg.data = tau[0];
        tau2_msg.data = tau[1];
        tau3_msg.data = tau[2];
        tau4_msg.data = tau[3];
        tau5_msg.data = tau[4];
        tau6_msg.data = tau[5];
        tau7_msg.data = tau[6];
        */
        tau_msg.data[0] = tau[0];
        tau_msg.data[1] = tau[1];
        tau_msg.data[2] = tau[2];
        tau_msg.data[3] = tau[3];
        tau_msg.data[4] = tau[4];
        tau_msg.data[5] = tau[5];
        tau_msg.data[6] = tau[6];

        // Publish
        // joint1_effort_pub.publish(tau1_msg);
        /*
        joint2_effort_pub.publish(tau2_msg);
        joint3_effort_pub.publish(tau3_msg);
        joint4_effort_pub.publish(tau4_msg);
        joint5_effort_pub.publish(tau5_msg);
        joint6_effort_pub.publish(tau6_msg);
        joint7_effort_pub.publish(tau7_msg);
        */
        //
        //joint_tor_pub.publish(tau_msg);
        //
        //pauseGazebo.call(pauseSrv);

#if DEBUG
        std::cout << "jacobian: " << std::endl << robot.getJacobian() << std::endl;
        std::cout << "jsim: " << std::endl << robot.getJsim() << std::endl;
        std::cout << "c: " << std::endl << robot.getCoriolis().transpose() << std::endl;
        std::cout << "g: " << std::endl << robot.getGravity().transpose() << std::endl;
        std::cout << "qd: " << std::endl << qd.data.transpose() << std::endl;
        std::cout << "q: " << std::endl << robot.getJntValues().transpose() << std::endl;
        std::cout << "tau: " << std::endl << tau.transpose() << std::endl;
        std::cout << "desired_pose: " << std::endl << des_pose << std::endl;
        std::cout << "current_pose: " << std::endl << robot.getCartesianPose() << std::endl;
#endif

        /*des_pose_init_state.request.link_state.pose.position.x = p.pos.x();
        des_pose_init_state.request.link_state.pose.position.y = p.pos.y();
        des_pose_init_state.request.link_state.pose.position.z = p.pos.z();
        if(des_pose_set_state_srv.call(des_pose_init_state))
            ROS_INFO("Desired pose state set.");
        else
            ROS_INFO("Failed to set desired pose state.");
        */
        // unpauseGazebo.call(pauseSrv);
        ros::spinOnce();
        // pauseGazebo.call(pauseSrv);
        loop_rate.sleep();
        
        // int n;
        // std::cin >> n;
        }
    }
    /*
    if(pauseGazebo.call(pauseSrv))
        ROS_INFO("Simulation paused.");
    else
        ROS_INFO("Failed to pause simulation.");
    */

    return 0;
}


//    std::vector<KDL::Frame> trajectory_frames;
//    KDL::Frame frame_1 = KDL::Frame(KDL::Rotation::EulerZYX(0,0,0),KDL::Vector(0.45,0.2,0.5));
//    KDL::Frame frame_2 = KDL::Frame(KDL::Rotation::EulerZYX(0,1.57,0),KDL::Vector(0.65,0.2,0.5));
//    KDL::Frame frame_3 = KDL::Frame(KDL::Rotation::EulerZYX(0,0,0),KDL::Vector(0.65,-0.2,0.5));
//    trajectory_frames.push_back(frame_1);
//    trajectory_frames.push_back(frame_2);
//    trajectory_frames.push_back(frame_3);
//    planner.CreateTrajectoryFromFrames(trajectory_frames,0.1,0.1);

//    KDL::Vector center(0.0,0.0,init_cart_pose.p.z());
//    KDL::Frame end_cart_pose = KDL::Frame(
//                init_cart_pose.M,
//                KDL::Vector(init_cart_pose.p.x(),
//                            -init_cart_pose.p.y(),
//                            init_cart_pose.p.z()));
//    planner.createCircPath(init_cart_pose,
//                           center,
//                           end_cart_pose.p,
//                           end_cart_pose.M,
//                           1.57,
//                           0.02);
// trajectory_point p = planner.compute_trajectory(0.0,4.0,1.0,init_position,end_position);
//    KDL::Trajectory* traj;
//    traj = planner.getTrajectory();
