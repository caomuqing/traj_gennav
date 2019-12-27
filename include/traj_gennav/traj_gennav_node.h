#ifndef TRAJ_GENNAV_NODE_H
#define TRAJ_GENNAV_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include "traj_gennav/in_loop_cmd_generator.h"
#include "traj_gennav/geometry_math_type.h"
#include "traj_gennav/poly_traj_generator.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include "traj_gennav/ExecutePath.h"
#include "traj_gennav/ReadPathFromFile.h"
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <geodetic_utils/geodetic_conv.hpp>
#include <std_srvs/Empty.h>


//ros::Publisher rpyt_command_pub;
ros::Publisher trajectory_sim_pub, trajectory_pub;
fullstate_t cmd_, current_odom_;
ros::ServiceServer read_service_;
ros::ServiceServer execute_service_;


bool readFileCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
bool executePathCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
void addIntermediateWaypoints();
void addCurrentOdometryWaypoint();
Eigen::Vector3d getPosPoly(Eigen::MatrixXd polyCoeff, int k, double t);

std::vector<double> easting;
std::vector<double> northing;
std::vector<double> height;
std::vector<double> heading;

// Parameters.
// GPS/ENU coordinates.
std::string coordinate_type_;
// Trajectory/poses command publisher.
std::string path_mode_;
// Heading alignment method.
std::string heading_mode_;
std::string mav_name_;
std::string frame_id_;
// Addition of intermediate command poses.
bool intermediate_poses_;
// Maximum distance between poses [m].
double intermediate_pose_separation_;
// Maximum speed (m/s).
double reference_speed_;
// Maximum acceleration (m/s^2).
double reference_acceleration_;
// Height for takeoff command [m].
double takeoff_height_;
// Height for landing command [m].
double landing_height_;
int dev_order_;


// A list of waypoints to visit.
// [x,y,z,heading]
std::vector<flatstate_t> initial_waypoints_;
Eigen::VectorXd polyTime_;
Eigen::MatrixXd polyCoeff_;

bool got_odom_ = false;
bool trajectory_obtained_ = false;
bool start_trajectory_ = false;
bool trajectory_wip_ = false;
ros::Time trajectory_start_time_;

size_t current_leg_;

const double kIntermediatePoseTolerance = 0.5;

geodetic_converter::GeodeticConverter geodetic_converter_;



void TimerCallback(const ros::TimerEvent&);
// void trajectory_cb(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg);
void odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
void geoVec3toEigenVec3 (geometry_msgs::Vector3 geoVector3, Eigen::Vector3f& eigenVec3);
void geoVec3toEigenVec3 (geometry_msgs::Vector3 geoVector3, Eigen::Vector3d& eigenVec3);
void geoPt3toEigenVec3 (geometry_msgs::Point geoPt3, Eigen::Vector3f& eigenVec3);
void geoPt3toEigenVec3 (geometry_msgs::Point geoPt3, Eigen::Vector3d& eigenVec3);
Eigen::VectorXd timeAllocation(Eigen::MatrixXd Path);

// Eigen::Vector3f prtcontrol(fullstate_t& cmd, fullstate_t& current);

Eigen::Vector3f CtrlOmega(1.0, 1.0, 1.3); //norminal natural frequency
Eigen::Vector3f CtrlEpsilon(1, 1, 1); //tuning parameter
Eigen::Vector3f CtrlZita(1, 1, 1.1); //damping ratio
Eigen::Vector3f PosErrorAccumulated_(0, 0, 0);
Eigen::Vector3f k_I_(0.02, 0.02, 0.2);
mppi_control::InLoopCmdGen InLoopCmdGen_(0.4);
float posErrAccLimit_ = 15.0, acc_xy_limit_=5.0;
float k_p_yaw_=0.02;

#endif  // TRAJ_GENNAV_NODE_H
