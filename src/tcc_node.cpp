#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include "tcc/in_loop_cmd_generator.h"
#include "tcc/geometry_math_type.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <nav_msgs/Odometry.h>

ros::Publisher rpyt_command_pub;
fullstate_t cmd_, current_;

void trajectory_cb(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg);
void odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
void geoVec3toEigenVec3 (geometry_msgs::Vector3 geoVector3, Eigen::Vector3f& eigenVec3);
void geoPt3toEigenVec3 (geometry_msgs::Point geoPt3, Eigen::Vector3f& eigenVec3);
Eigen::Vector3f prtcontrol(fullstate_t& cmd, fullstate_t& current);

Eigen::Vector3f CtrlOmega(0.54, 0.62, 0.78); //norminal natural frequence
Eigen::Vector3f CtrlEpsilon(1, 1, 1); //tuning parameter
Eigen::Vector3f CtrlZita(1, 1, 1.1); //damping ratio
Eigen::Vector3f PosErrorAccumulated_(0, 0, 0);
Eigen::Vector3f k_I_(0, 0, 0);
mppi_control::InLoopCmdGen InLoopCmdGen_(0.5);
float k_p_yaw_=0.02;

int main(int argc, char** argv){
  ros::init(argc, argv, "tcc");

  ros::NodeHandle nh("~");

  rpyt_command_pub = nh.advertise<mav_msgs::RollPitchYawrateThrust>(
                                    "/firefly/command/roll_pitch_yawrate_thrust", 50);

  ros::Subscriber trajectory_sub = nh.subscribe<trajectory_msgs::MultiDOFJointTrajectory>(
                                   "/firefly/command/trajectory", 10, trajectory_cb);
  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/vins_estimator/odometry", 10, odom_cb);
  ros::spin();

  return 0;
}

void trajectory_cb(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg)
{
  cmd_.timestamp = ros::Time::now();;
  geoVec3toEigenVec3(msg->points[0].transforms[0].translation, cmd_.pos);
  geoVec3toEigenVec3(msg->points[0].velocities[0].linear, cmd_.vel);
  geoVec3toEigenVec3(msg->points[0].accelerations[0].linear, cmd_.acc);
  Eigen::Quaternion<float> cmd_Quat(msg->points[0].transforms[0].rotation.w, 
                                    msg->points[0].transforms[0].rotation.x, 
                                    msg->points[0].transforms[0].rotation.y, 
                                    msg->points[0].transforms[0].rotation.z);
  get_dcm_from_q(cmd_.R, cmd_Quat);
  std::cout<<"trarget pos x: "<<cmd_.pos(0)<<"y: "<<cmd_.pos(1)<<"z: "<<cmd_.pos(2)<<"\n";

}

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_.timestamp = ros::Time::now();
  geoPt3toEigenVec3(msg->pose.pose.position, current_.pos);
  geoVec3toEigenVec3(msg->twist.twist.linear, current_.vel);
  Eigen::Quaternion<float> current_Quat(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, 
                                        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  get_dcm_from_q(current_.R, current_Quat);

  if ((current_.timestamp - cmd_.timestamp).toSec() < 0.05){
    //do control
    Eigen::Vector3f tarAcc_ = prtcontrol(cmd_, current_);
    Eigen::Vector3f TrajAtti, tarAtti, currentAtti;
    get_euler_from_R(TrajAtti, cmd_.R);
    get_euler_from_R(currentAtti, current_.R);
    mppi_control::InLoopCmdGen::drone_cmd_t in_loop_cmd = InLoopCmdGen_.cal_R_T(tarAcc_, current_.R, TrajAtti(2));
    get_euler_from_R(tarAtti, in_loop_cmd.R);
    mav_msgs::RollPitchYawrateThrust rpyrt_msg;
    rpyrt_msg.roll = tarAtti(0);
    rpyrt_msg.pitch = tarAtti(1);
    rpyrt_msg.yaw_rate = wrapPi(tarAtti(2) - currentAtti(2)) * k_p_yaw_;
    rpyrt_msg.thrust.x = in_loop_cmd.T;
    rpyt_command_pub.publish(rpyrt_msg);

  }
}

void geoVec3toEigenVec3 (geometry_msgs::Vector3 geoVector3, Eigen::Vector3f& eigenVec3)
{
  eigenVec3(0) = geoVector3.x;
  eigenVec3(1) = geoVector3.y;
  eigenVec3(2) = geoVector3.z;
}

void geoPt3toEigenVec3 (geometry_msgs::Point geoPt3, Eigen::Vector3f& eigenVec3)
{
  eigenVec3(0) = geoPt3.x;
  eigenVec3(1) = geoPt3.y;
  eigenVec3(2) = geoPt3.z;
}
// void geoQuatoEigenQua (geometry_msgs::Quaternion geoQua, Eigen::Quaternion& eigenQua)
// {
//   eigenQua = Quaternion(geoQua.w, geoQua.x, geoQua.y, geoQua.z);
// }

Eigen::Vector3f prtcontrol(fullstate_t& cmd, fullstate_t& current)
{ 
  PosErrorAccumulated_ += (cmd.pos - current.pos);
  Eigen::Vector3f tarAcc;
  tarAcc(0) = pow(CtrlOmega(0)/CtrlEpsilon(0), 2)*(cmd.pos(0)-current.pos(0)) +
             2*CtrlZita(0)*CtrlOmega(0)/CtrlEpsilon(0)*(cmd.vel(0)-current.vel(0)) + 
             cmd.acc(0) + k_I_(0)*PosErrorAccumulated_(0);
  tarAcc(1) = pow(CtrlOmega(1)/CtrlEpsilon(1), 2)*(cmd.pos(1)-current.pos(1)) +
             2*CtrlZita(1)*CtrlOmega(1)/CtrlEpsilon(1)*(cmd.vel(1)-current.vel(1)) + 
             cmd.acc(1) + k_I_(1)*PosErrorAccumulated_(1);
  tarAcc(2) = pow(CtrlOmega(2)/CtrlEpsilon(2), 2)*(cmd.pos(2)-current.pos(2)) +
             2*CtrlZita(2)*CtrlOmega(2)/CtrlEpsilon(2)*(cmd.vel(2)-current.vel(2)) + 
             cmd.acc(2) +  + k_I_(2)*PosErrorAccumulated_(2);;
  return tarAcc;
}