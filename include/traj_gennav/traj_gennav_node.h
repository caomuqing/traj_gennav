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
//#include <geodetic_utils/geodetic_conv.hpp>
#include <std_srvs/Empty.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
// #include <rviz_visual_tools/rviz_visual_tools.h>
#include "Plane.h"
#include <nav_msgs/Path.h>

// using namespace rviz_visual_tools;
// rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

//ros::Publisher rpyt_command_pub;
ros::Publisher trajectory_sim_pub, trajectory_pub;
fullstate_t cmd_, current_odom_;
ros::ServiceServer read_service_;
ros::ServiceServer execute_service_;
ros::Publisher Wall_estimate_pub;
ros::Publisher nominal_path_publisher;
ros::Publisher odom_path_publisher;

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
std::string sim_type_;
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
int nSeg_wip_ = 0;

// A list of waypoints to visit.
// [x,y,z,heading]
std::vector<flatstate_t> initial_waypoints_;
Eigen::VectorXd polyTime_;
Eigen::MatrixXd polyCoeff_;

bool got_odom_ = false;
bool trajectory_obtained_ = false;
bool start_trajectory_ = false;
bool trajectory_wip_ = false;
bool idle_state_ = false;
ros::Time trajectory_start_time_(0.001);

size_t current_leg_;

const double kIntermediatePoseTolerance = 1.0;
double kDistanceFromBuilding = 5.0;

//geodetic_converter::GeodeticConverter geodetic_converter_;

Eigen::Vector3d planeWorldABC_(-1.0, 0.0, 0.0);
double planeWorldD_ = 0.0;
Eigen::Vector3d planeBodyABC_(0.0, 0.0, 0.0);
double planeBodyD_ = 0.0;
Eigen::Vector3d planeWorldABC_Est_(-1.0, 0.0, 0.0);
double planeWorldD_Est_ = 0.0;
double est_K_ = 0.7;
double desired_distance_s_ = 5.0;
bool path_from_message_=false;

ros::Time last_wall_msg_time_;
ros::Time last_odom_pub_time_(0);
bool wall_msg_updated_;
bool start_command_prev_;

nav_msgs::Path odom_path_visualizer_;

void TimerCallback(const ros::TimerEvent&);
void ReplanTimerCallback(const ros::TimerEvent&);
// void trajectory_cb(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg);
void odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
void wall_cb(const std_msgs::Float32MultiArray::ConstPtr& msg);
void path_input_cb(const nav_msgs::Path::ConstPtr& msg);
void start_path_command_cb(const std_msgs::Bool::ConstPtr& msg);

void geoVec3toEigenVec3 (geometry_msgs::Vector3 geoVector3, Eigen::Vector3f& eigenVec3);
void geoVec3toEigenVec3 (geometry_msgs::Vector3 geoVector3, Eigen::Vector3d& eigenVec3);
void geoPt3toEigenVec3 (geometry_msgs::Point geoPt3, Eigen::Vector3f& eigenVec3);
void geoPt3toEigenVec3 (geometry_msgs::Point geoPt3, Eigen::Vector3d& eigenVec3);
Eigen::VectorXd timeAllocation(Eigen::MatrixXd Path);
trajectory_msgs::MultiDOFJointTrajectory generateTrajOnline(double timeinTraj, double T_s, int horizon);
trajectory_msgs::MultiDOFJointTrajectory generateTrajNominal(double timeinTraj, double T_s, int horizon);

void sampleWholeTrajandVisualize();

#endif  // TRAJ_GENNAV_NODE_H
