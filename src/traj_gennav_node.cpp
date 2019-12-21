#include <traj_gennav/traj_gennav_node.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "traj_gennav");

  ros::NodeHandle nh("~");

  if(!nh.getParam("coordinate_type", coordinate_type_) ||
    !nh.getParam("path_mode", path_mode_) ||
    !nh.getParam("heading_mode", heading_mode_) ||
    !nh.getParam("reference_speed", reference_speed_) ||
    !nh.getParam("reference_acceleration", reference_acceleration_) ||
    !nh.getParam("takeoff_height", takeoff_height_) ||
    !nh.getParam("landing_height", landing_height_) ||
    !nh.getParam("frame_id", frame_id_) ||
    !nh.getParam("intermediate_poses", intermediate_poses_)||
    !nh.getParam("easting", easting) ||
    !nh.getParam("northing", northing) ||
    !nh.getParam("height", height))
    ROS_WARN("Error loading parameters!");

  nh.param("derivative_order", dev_order_, 3);
  if (coordinate_type_ == "gps" || coordinate_type_ == "enu") {
  } else {
    ROS_WARN("Unknown coordinate type - please enter 'gps' or 'enu'.");
  }

  if (path_mode_ != "poses" && path_mode_ != "polynomial") {
    ROS_WARN("Unknown path type - please enter 'poses', or 'trajectory'.");
  }

  if (heading_mode_ != "auto" && heading_mode_ != "manual" && heading_mode_ != "zero") {
    ROS_WARN("Unknown heading alignment mode - please enter 'auto', 'manual', or 'zero'.");
  }

  if (intermediate_poses_) {
    if(!nh.getParam("intermediate_pose_separation", intermediate_pose_separation_))
      ROS_WARN("Cannot set intermediate poses without an intermediate pose separation.");
  }

  if (heading_mode_ == "manual" && !nh.getParam("heading", heading)) {
    ROS_WARN("Heading in manual mode is unspecified!");
  }
  // Check for valid trajectory inputs.
  if (!(easting.size() == northing.size() && northing.size() == height.size())) {
    ROS_WARN("Error: path parameter arrays are not the same size");
  }
  if (heading_mode_ == "manual" && !(height.size() == heading.size())) {
    ROS_WARN("Error: path parameter arrays are not the same size");
  }

  rpyt_command_pub = nh.advertise<mav_msgs::RollPitchYawrateThrust>(
                                    "/firefly/command/roll_pitch_yawrate_thrust1", 50);
  trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
                                    "/command/trajectory", 50);

  ros::Subscriber trajectory_sub = nh.subscribe<trajectory_msgs::MultiDOFJointTrajectory>(
                                   "/firefly/command/trajectory", 10, trajectory_cb);
  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/vins_estimator/odometry", 10, odom_cb);

  ros::Timer timer = nh.createTimer(ros::Duration(1.0/10), TimerCallback);

  read_service_ = nh.advertiseService("readfile", readFileCallback);

  execute_service_ = nh.advertiseService("execute_path", executePathCallback);

  trajectory_sim_pub = nh.advertise<nav_msgs::Odometry>("/trajectory_sim", 50);
  ros::spin();

  return 0;
}

void trajectory_cb(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg)
{
  cmd_.timestamp = ros::Time::now();
  geoVec3toEigenVec3(msg->points[0].transforms[0].translation, cmd_.pos);
  geoVec3toEigenVec3(msg->points[0].velocities[0].linear, cmd_.vel);
  geoVec3toEigenVec3(msg->points[0].accelerations[0].linear, cmd_.acc);
  Eigen::Quaternion<double> cmd_Quat(msg->points[0].transforms[0].rotation.w, 
                                    msg->points[0].transforms[0].rotation.x, 
                                    msg->points[0].transforms[0].rotation.y, 
                                    msg->points[0].transforms[0].rotation.z);
  get_dcm_from_q(cmd_.R, cmd_Quat);
  //std::cout<<"trarget pos x: "<<cmd_.pos(0)<<"y: "<<cmd_.pos(1)<<"z: "<<cmd_.pos(2)<<"\n";
  ROS_INFO_ONCE("Got first trajectory message!");

}

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
  got_odom_ = true;
  current_odom_.timestamp = ros::Time::now();
  geoPt3toEigenVec3(msg->pose.pose.position, current_odom_.pos);
  geoVec3toEigenVec3(msg->twist.twist.linear, current_odom_.vel);
  Eigen::Quaternion<double> current_Quat(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, 
                                        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  get_dcm_from_q(current_odom_.R, current_Quat);
  ROS_INFO_ONCE("Got first odom message!");
}

// void geoQuatoEigenQua (geometry_msgs::Quaternion geoQua, Eigen::Quaternion& eigenQua)
// {
//   eigenQua = Quaternion(geoQua.w, geoQua.x, geoQua.y, geoQua.z);
// }

// Eigen::Vector3f prtcontrol(fullstate_t& cmd, fullstate_t& current)
// { 
//   PosErrorAccumulated_ += (cmd.pos - current.pos);
//   for (int i =0; i<=2; i++){
//     if (PosErrorAccumulated_(i)>posErrAccLimit_) PosErrorAccumulated_(i) = posErrAccLimit_;
//     else if (PosErrorAccumulated_(i)<-posErrAccLimit_) PosErrorAccumulated_(i) = -posErrAccLimit_;
//   }
//   Eigen::Vector3f tarAcc;
//   tarAcc(0) = pow(CtrlOmega(0)/CtrlEpsilon(0), 2)*(cmd.pos(0)-current.pos(0)) +
//              2*CtrlZita(0)*CtrlOmega(0)/CtrlEpsilon(0)*(cmd.vel(0)-current.vel(0)) + 
//              cmd.acc(0) + k_I_(0)*PosErrorAccumulated_(0);
//   tarAcc(1) = pow(CtrlOmega(1)/CtrlEpsilon(1), 2)*(cmd.pos(1)-current.pos(1)) +
//              2*CtrlZita(1)*CtrlOmega(1)/CtrlEpsilon(1)*(cmd.vel(1)-current.vel(1)) + 
//              cmd.acc(1) + k_I_(1)*PosErrorAccumulated_(1);
//   tarAcc(2) = pow(CtrlOmega(2)/CtrlEpsilon(2), 2)*(cmd.pos(2)-current.pos(2)) +
//              2*CtrlZita(2)*CtrlOmega(2)/CtrlEpsilon(2)*(cmd.vel(2)-current.vel(2)) + 
//              cmd.acc(2) +  + k_I_(2)*PosErrorAccumulated_(2) + ONE_G;

//   if (tarAcc(0) >= acc_xy_limit_) tarAcc(0) = acc_xy_limit_;
//   else if (tarAcc(0) < -acc_xy_limit_) tarAcc(0) = -acc_xy_limit_;
//   if (tarAcc(1) >= acc_xy_limit_) tarAcc(1) = acc_xy_limit_;
//   else if (tarAcc(1) < -acc_xy_limit_) tarAcc(1) = -acc_xy_limit_;
//   if (tarAcc(2) >= 2.0*ONE_G) tarAcc(2) = 2.0*ONE_G;
//   else if (tarAcc(2) < 0.3*ONE_G) tarAcc(2) = 0.3*ONE_G;
//   std::cout<<"trarget acc x: "<<tarAcc(0)<<"y: "<<tarAcc(1)<<"z: "<<tarAcc(2)<<"\n";

//   return tarAcc;
// }

void TimerCallback(const ros::TimerEvent&)
{
  if (got_odom_){
    if ((ros::Time::now()-current_odom_.timestamp).toSec()>0.8){
      got_odom_ = false;
    }
  }
  if (start_trajectory_){
    if (trajectory_wip_){
      double timeElasped = (ros::Time::now() - trajectory_start_time_).toSec();
      if (timeElasped > polyTime_.sum()){
        ROS_INFO("Finished publishing trajectory.");
        trajectory_wip_ = false;
        start_trajectory_ = false;
      } else {  //find which segment it is at and publish accordingly
        int _nSeg = polyTime_.size();
        int _nSeg_wip;
        double _timeSum = 0.0;
        for (int k=0; k<_nSeg; k++){
          if (timeElasped < _timeSum +polyTime_[k]){
            _nSeg_wip = k;
            break;
          } else _timeSum += polyTime_[k];
        }
        std::cout<<"now in "<<_nSeg_wip<<" number of segment, time elapsed is "<<timeElasped<<
              "time in this segment is"<< timeElasped-_timeSum<<"\n";
        Eigen::Vector3d _pos, _vel, _acc;
        double _yaw = 0.0;
        _pos.setZero();
        for (int i=0; i<3; i++){
          for (int k=0; k<2*dev_order_; k++){
            _pos(i) += polyCoeff_(_nSeg_wip, i*2*dev_order_+ k) * std::pow(timeElasped-_timeSum, k);
            _vel(i) += (k>0? k*
                        polyCoeff_(_nSeg_wip, i*2*dev_order_+ k) * std::pow(timeElasped-_timeSum, k-1):0);
            _acc(i) += (k>1? k*(k-1)*
                        polyCoeff_(_nSeg_wip, i*2*dev_order_+ k) * std::pow(timeElasped-_timeSum, k-2):0);
          }
        }
        for (int k=0; k<2*dev_order_; k++){
          _yaw += polyCoeff_(_nSeg_wip, 3*2*dev_order_+ k) * std::pow(timeElasped-_timeSum, k);
        }
        _yaw = wrapPi(_yaw);        
        trajectory_msgs::MultiDOFJointTrajectory trajset_msg;
        trajectory_msgs::MultiDOFJointTrajectoryPoint trajpt_msg;
        geometry_msgs::Transform transform_msg;
        geometry_msgs::Twist accel_msg, vel_msg;
        transform_msg.translation.x = _pos(0);
        transform_msg.translation.y = _pos(1);
        transform_msg.translation.z = _pos(2);
        transform_msg.rotation.x = 0;
        transform_msg.rotation.y = 0;
        transform_msg.rotation.z = sinf(_yaw*0.5);
        transform_msg.rotation.w = cosf(_yaw*0.5);
        trajpt_msg.transforms.push_back(transform_msg);
        vel_msg.linear.x = _vel(0);
        vel_msg.linear.y = _vel(1);
        vel_msg.linear.z = _vel(2);
        accel_msg.linear.x = _acc(0);
        accel_msg.linear.x = _acc(1);
        accel_msg.linear.x = _acc(2);
        trajpt_msg.velocities.push_back(vel_msg);
        trajpt_msg.accelerations.push_back(accel_msg);
        trajset_msg.points.push_back(trajpt_msg);
        trajectory_pub.publish(trajset_msg);
        
        // for testing
        //_pos = getPosPoly(polyCoeff_, _nSeg_wip, timeElasped-_timeSum);
        nav_msgs::Odometry trajectory_odom;
        trajectory_odom.header.stamp = ros::Time::now();
        trajectory_odom.header.frame_id = frame_id_;
        trajectory_odom.pose.pose.position.x = _pos(0);
        trajectory_odom.pose.pose.position.y = _pos(1);
        trajectory_odom.pose.pose.position.z = _pos(2);
        trajectory_odom.pose.pose.orientation.x = 0;
        trajectory_odom.pose.pose.orientation.y = 0;
        trajectory_odom.pose.pose.orientation.z = sinf(_yaw*0.5);
        trajectory_odom.pose.pose.orientation.w = cosf(_yaw*0.5);

        trajectory_sim_pub.publish(trajectory_odom);
      }

    } else {
      trajectory_start_time_ = ros::Time::now();
      trajectory_wip_ = true;
    }
  }
}

bool readFileCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  // if (!got_odom_){
  //   ROS_WARN("NO ODOM, WILL NOT CALCULATE TRAJECTORY");
  //   return false;
  // }
  initial_waypoints_.clear();
  addCurrentOdometryWaypoint();

  // Add (x,y,z) co-ordinates from file to path.
  for (size_t i = 0; i < easting.size(); i++) {
    flatstate_t cwp;
    // GPS path co-ordinates.
    if (coordinate_type_ == "gps") {
      double initial_latitude;
      double initial_longitude;
      double initial_altitude;

      // Convert GPS point to ENU co-ordinates.
      // NB: waypoint altitude = desired height above reference + registered
      // reference altitude.
      geodetic_converter_.getReference(&initial_latitude, &initial_longitude, &initial_altitude);
      geodetic_converter_.geodetic2Enu(northing[i], easting[i], (initial_altitude + height[i]),
                                       &cwp.pos(0), &cwp.pos(1), &cwp.pos(2));
    }
    // ENU path co-ordinates.
    else if (coordinate_type_ == "enu") {
      cwp.pos(0) = easting[i];
      cwp.pos(1) = northing[i];
      cwp.pos(2) = height[i];
    }
    initial_waypoints_.push_back(cwp);
  }

  // Add heading from file to path.
  for (size_t i = 1; i < initial_waypoints_.size(); i++) {
    if (heading_mode_ == "manual") {
      initial_waypoints_[i].yaw = heading[i] * (M_PI / 180.0);
    } else if (heading_mode_ == "auto") {
      // Compute heading in direction towards next point.
      initial_waypoints_[i].yaw = atan2(initial_waypoints_[i].pos(1) - initial_waypoints_[i - 1].pos(1),
                                        initial_waypoints_[i].pos(0) - initial_waypoints_[i - 1].pos(0));
    } else if (heading_mode_ == "zero") {
      initial_waypoints_[i].yaw = 0.0;
    }
  }

  // As first target point, add current (x,y) position, but with height at
  // that of the first requested waypoint, so that the MAV first adjusts height
  // moving only vertically.
  if (initial_waypoints_.size() >= 2) {
    flatstate_t vwp;
    vwp.pos = current_odom_.pos;
    vwp.pos(2) = initial_waypoints_[1].pos(2);
    if (heading_mode_ == "zero") {
      vwp.yaw = 0.0;
    } else if (heading_mode_ == "manual") { 
      vwp.yaw = initial_waypoints_[0].yaw;     // Do not change heading.
    }
    if ((vwp.pos-initial_waypoints_[1].pos).norm() > kIntermediatePoseTolerance)
      initial_waypoints_.insert(initial_waypoints_.begin() + 1, vwp);
  }

  // Limit maximum distance between waypoints.
  if (intermediate_poses_) {
    addIntermediateWaypoints();
  }

  int num_wp = (int)initial_waypoints_.size();
  ROS_INFO("Path loaded from file. Number of points in path: %d", num_wp);

  //conversion to matrix form
  Eigen::MatrixXd initial_path(num_wp,4);  
  initial_path(0, 3) = initial_waypoints_[0].yaw;
  for (int i=0; i<num_wp; i++){
    initial_path(i, 0) = initial_waypoints_[i].pos(0);
    initial_path(i, 1) = initial_waypoints_[i].pos(1);
    initial_path(i, 2) = initial_waypoints_[i].pos(2);

    if (i+1<num_wp)
      initial_path(i+1, 3) = initial_path(i, 3) + 
                            wrapPi(initial_waypoints_[i+1].yaw - initial_path(i, 3));
  }

  std::cout<<"initial_path is "<<initial_path<<"\n";
  TrajectoryGeneratorWaypoint  trajectoryGeneratorWaypoint;
  
  Eigen::MatrixXd vel = Eigen::MatrixXd::Zero(2, 4);  //start and end velocity as zero
  Eigen::MatrixXd acc = Eigen::MatrixXd::Zero(2, 4);  //start and end acceleration as zero

  // give an arbitraty time allocation, all set all durations as 1 in the commented function.
  ROS_INFO("Start solving for trajectory...");
  polyTime_  = timeAllocation(initial_path);
  polyCoeff_ = trajectoryGeneratorWaypoint.PolyQPGenerationClosedForm(dev_order_, initial_path, vel, acc, polyTime_);

  std::cout<<"polyTime_ is "<<polyTime_<<"\n";
  std::cout<<"polyCoeff_ is"<<polyCoeff_<<"\n";
  trajectory_obtained_ = true;

  current_leg_ = 0;
  return true;
}

bool executePathCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  // if (!got_odom_){
  //   ROS_WARN("NO ODOM, WILL NOT CALCULATE TRAJECTORY");
  //   return false;
  // }

  if (trajectory_obtained_){
    start_trajectory_ = true;
    return true;
  } else return false;
  
}

void addCurrentOdometryWaypoint() {
  flatstate_t vwp;
  vwp.pos= current_odom_.pos;
  Eigen::Vector3d rpy;
  get_euler_from_R(rpy, current_odom_.R);
  vwp.yaw = rpy(2);
  initial_waypoints_.push_back(vwp);
}

void addIntermediateWaypoints() {
  for (size_t i = 1; i < initial_waypoints_.size(); ++i) {
    flatstate_t wpa = initial_waypoints_[i - 1];
    flatstate_t wpb = initial_waypoints_[i];
    double dist = (wpa.pos - wpb.pos).norm();

    // Minimum tolerance between points set to avoid subsequent numerical errors
    // in trajectory optimization.
    while (dist > intermediate_pose_separation_ + kIntermediatePoseTolerance) {
      flatstate_t iwp;
      iwp.pos(0) = wpa.pos(0) +
                  (intermediate_pose_separation_ / dist) * (wpb.pos(0) - wpa.pos(0));
      iwp.pos(1) = wpa.pos(1) +
                  (intermediate_pose_separation_ / dist) * (wpb.pos(1) - wpa.pos(1));
      iwp.pos(2) = wpa.pos(2) +
                  (intermediate_pose_separation_ / dist) * (wpb.pos(2) - wpa.pos(2));
      iwp.yaw = wpb.yaw;
      initial_waypoints_.insert(initial_waypoints_.begin() + i, iwp);
      wpa = iwp;
      dist = (wpa.pos - wpb.pos).norm();
      i++;
    }
  }
}

Eigen::VectorXd timeAllocation(Eigen::MatrixXd Path)
{ 
  Eigen::VectorXd time(Path.rows() - 1);
  
  float time_to_max_v = reference_speed_ /reference_acceleration_;
  float dist_to_max_v = 0.5 * reference_acceleration_ * time_to_max_v * time_to_max_v;
  //float dist_to_zero_v = _Vel * time_to_max_v - 0.5 * _Acc * time_to_max_v * time_to_max_v;
  
  for (int i=0; i<Path.rows()-1; i++){
      float distance = (Path.row(i) - Path.row(i+1)).norm();
      if (distance > dist_to_max_v*2){
          time(i) = 2 * time_to_max_v + (distance - dist_to_max_v*2)/reference_speed_;
      } else {
          time(i) = 2*sqrt(distance/reference_acceleration_);
      }
  }

  return time;
}

Eigen::Vector3d getPosPoly(Eigen::MatrixXd polyCoeff, int k, double t)
{
    Eigen::Vector3d ret;

    for ( int dim = 0; dim < 3; dim++ ){
        Eigen::VectorXd coeff = (polyCoeff.row(k)).segment( dim * 2*dev_order_, 2*dev_order_ );
        Eigen::VectorXd time  = Eigen::VectorXd::Zero(2*dev_order_);
        
        for(int j = 0; j < 2*dev_order_; j ++)
          if(j==0)
              time(j) = 1.0;
          else
              time(j) = pow(t, j);

        ret(dim) = coeff.dot(time);
        //cout << "dim:" << dim << " coeff:" << coeff << endl;
    }

    return ret;
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

void geoVec3toEigenVec3 (geometry_msgs::Vector3 geoVector3, Eigen::Vector3d& eigenVec3)
{
  eigenVec3(0) = geoVector3.x;
  eigenVec3(1) = geoVector3.y;
  eigenVec3(2) = geoVector3.z;
}

void geoPt3toEigenVec3 (geometry_msgs::Point geoPt3, Eigen::Vector3d& eigenVec3)
{
  eigenVec3(0) = geoPt3.x;
  eigenVec3(1) = geoPt3.y;
  eigenVec3(2) = geoPt3.z;
}