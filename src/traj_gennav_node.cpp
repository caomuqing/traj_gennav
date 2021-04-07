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
  nh.param("desired_distance", desired_distance_s_, 5.0);
  nh.param("path_from_message", path_from_message_, false);

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

  if (!nh.getParam("sim_type", sim_type_)){
    ROS_WARN("Don't have sim type parameter. Exiting");
    exit(-1);
  }

  if(sim_type_!="rotors" && sim_type_!="dji" && sim_type_!="none" && sim_type_!="none_st" && sim_type_!="sim_st"){
    ROS_WARN("not good, don't know in simulation or real flight");
    exit(-1);
  }

  // rpyt_command_pub = nh.advertise<mav_msgs::RollPitchYawrateThrust>(
  //                                   "/firefly/command/roll_pitch_yawrate_thrust1", 50);
  trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
                                    "/firefly/command/trajectory", 50);

  //ros::Subscriber trajectory_sub = nh.subscribe<trajectory_msgs::MultiDOFJointTrajectory>(
  //                                 "/firefly/command/trajectory", 10, trajectory_cb);
  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/vins_estimator/odometry", 10, odom_cb);

  ros::Subscriber wall_sub = nh.subscribe<std_msgs::Float32MultiArray>("/Environment_seg/Wall_model_coefficient", 10, wall_cb);

  ros::Subscriber path_input_sub = nh.subscribe<nav_msgs::Path>("/NTU_internal/path_in_local_ref", 10, path_input_cb);

  ros::Subscriber start_path_command_sub = nh.subscribe<std_msgs::Bool>("/st_sdk/start_following_path", 10, start_path_command_cb);

  ros::Timer timer = nh.createTimer(ros::Duration(1.0/25), TimerCallback);
  ros::Timer replantimer = nh.createTimer(ros::Duration(1.0/5), ReplanTimerCallback);

  read_service_ = nh.advertiseService("readfile", readFileCallback);

  execute_service_ = nh.advertiseService("execute_path", executePathCallback);

  trajectory_sim_pub = nh.advertise<nav_msgs::Odometry>("/trajectory_sim", 50);
  Wall_estimate_pub = nh.advertise<geometry_msgs::PoseStamped> ("/wall_estimate", 1);
  nominal_path_publisher = nh.advertise<nav_msgs::Path> ("/nominal_path", 1);
  odom_path_publisher = nh.advertise<nav_msgs::Path> ("/odom_path", 1);

  visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("/world","/plane_world"));
  visual_tools_->loadMarkerPub();  // create publisher before waiting
  visual_tools_->deleteAllMarkers();
  visual_tools_->enableBatchPublishing();

  ros::spin();

  return 0;
}

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
  got_odom_ = true;
  current_odom_.timestamp = ros::Time::now();
  geoPt3toEigenVec3(msg->pose.pose.position, current_odom_.pos);

  if (sim_type_!="rotors"){ // when velocity from odom is in ENU frame
    geoVec3toEigenVec3(msg->twist.twist.linear, current_odom_.vel);
  } else { // when velocity from odom is in body frame
    Eigen::Quaterniond orientation_W_B(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, 
                                       msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Eigen::Vector3d velocity_body(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    Eigen::Vector3d velocity_world = orientation_W_B *velocity_body;    
    current_odom_.vel = velocity_world;
  }


  Eigen::Quaternion<double> current_Quat(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, 
                                        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  get_dcm_from_q(current_odom_.R, current_Quat);
  ROS_INFO_ONCE("Got first odom message!");

  // if (true){ //st sim testing
  //   tf::Quaternion q0(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 
  //                     msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  //   tf::Quaternion q1 = tf::createQuaternionFromRPY(3.1415927, 0, -3.1415927/2); 
  //   tf::Quaternion qf = q1*q0;
  //   Eigen::Quaternion<double> current_Quat1(qf.w(), qf.x(), qf.y(), qf.z());
  //   get_dcm_from_q(current_odom_.R, current_Quat1);   
  // }
  if (sim_type_=="none_st"||sim_type_=="sim_st"){
    Eigen::Vector3d _rpy;
    get_euler_from_R(_rpy, current_odom_.R);
    tf::Quaternion qf = tf::createQuaternionFromRPY(_rpy(0), -_rpy(1), -wrapPi(_rpy(2)-3.1415927/2)); 
    Eigen::Quaternion<double> current_Quat1(qf.w(), qf.x(), qf.y(), qf.z());
    get_dcm_from_q(current_odom_.R, current_Quat1);
    get_euler_from_R(_rpy, current_odom_.R);
    //std::cout<< " CURRENT ENU roll"<< _rpy(0)/3.1415*180<<"  pitch"<<_rpy(1)/3.1415*180<<"  yaw"<<_rpy(2)/3.1415*180<<"\n";
  }

  if ((ros::Time::now()-last_odom_pub_time_).toSec()>0.15){
    //publish odom path for visualization
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.header.frame_id = frame_id_;
    this_pose_stamped.pose = msg->pose.pose;
    odom_path_visualizer_.header.stamp = ros::Time::now();
    odom_path_visualizer_.header.frame_id = frame_id_;
    odom_path_visualizer_.poses.push_back(this_pose_stamped);
    odom_path_publisher.publish(odom_path_visualizer_); 
    last_odom_pub_time_ = ros::Time::now();
  }


}

void wall_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  ROS_INFO_ONCE("Got first wall model message!");
  planeBodyABC_(0) = msg->data[0];
  planeBodyABC_(1) = msg->data[1];
  planeBodyABC_(2) = msg->data[2];
  planeBodyD_ = msg->data[3];
  last_wall_msg_time_ = ros::Time::now();
  wall_msg_updated_ = true;

  //rotate plane coefficients to world frame
  Eigen::MatrixXd _R_lidar_body(3,3);
  _R_lidar_body.setZero();
  _R_lidar_body(0,0) = 1;
  _R_lidar_body(1,1) = -1;
  _R_lidar_body(2,2) = -1;
  Eigen::Vector3d x_0_body(0, 0, -planeBodyD_/planeBodyABC_(2));
  Eigen::Vector3d x_0_world = (current_odom_.R) * 
                              (_R_lidar_body * x_0_body) + current_odom_.pos;
  planeWorldABC_ = (current_odom_.R * _R_lidar_body) * planeBodyABC_ ;
  planeWorldABC_ = planeWorldABC_/planeWorldABC_.norm(); //normalization
  planeWorldD_ = -planeWorldABC_.dot(x_0_world);     
  if (planeWorldABC_.dot(current_odom_.pos) + planeWorldD_ < 0){
    planeWorldABC_ = -planeWorldABC_;
    planeWorldD_ = -planeWorldD_;
  }                        

  //double planeWorldD = planeBodyD_ - planeBodyABC_.dot(current_odom_.pos);
  // visual_tools_->deleteAllMarkers();
  // visual_tools_->publishABCDPlane(planeWorldABC_(0), planeWorldABC_(1), planeWorldABC_(2), 
  //                                 planeWorldD_, rviz_visual_tools::BLUE, 50.0, 50.0);

  // Plane _facadePlane(planeWorldABC(0), planeWorldABC(1), planeWorldABC(2), planeWorldD);
  // Vec3f _curPos(current_odom_.pos(0), current_odom_.pos(1), current_odom_.pos(2));
  // Vec3f _facadeNormalXY(planeWorldABC(0), planeWorldABC(1), 0.0f); //we keep only xy direction
  // Vec3f _intersectPt = _facadePlane.IntersectLine(_curPos, _curPos +_facadeNormalXY);
  // Vec3f _projectPt = _intersectPt + Vec3f::Normalize(_curPos - _intersectPt) * kDistanceFromBuilding;
  // float _replanSurfaceD = -Vec3f::Dot(_facadeNormalXY, _projectPt);
  // visual_tools_->publishABCDPlane(_facadeNormalXY.x, _facadeNormalXY.y, _facadeNormalXY.z, 
  //                                 _replanSurfaceD, rviz_visual_tools::RED, 5.0, 5.0);
  // visual_tools_->trigger();  //publish plane for testing purpose only
}

void start_path_command_cb(const std_msgs::Bool::ConstPtr& msg)
{ 
  if (msg->data!=start_command_prev_){
    if (msg->data==true && trajectory_obtained_){ //command to start and got traj already
      start_trajectory_ = true;
      idle_state_ = false;
      ROS_INFO("STARTING TRAJECTORY PUBLISH!");
    } else if (msg->data==false){ //command change to stop traj
      start_trajectory_ = false;
      idle_state_ = true;      
    }
  }
  start_command_prev_ = msg->data;
}

void path_input_cb(const nav_msgs::Path::ConstPtr& msg) //for st
{
  if (!path_from_message_){
    ROS_WARN("DID NOT ALLOW PATH FROM MESSAGE!");
    return;
  }
  if (!got_odom_){
    ROS_WARN("NO ODOM, WILL NOT CALCULATE TRAJECTORY");
    return;
  }
  start_trajectory_ = false;
  idle_state_ = true;
  trajectory_wip_ = false;
  initial_waypoints_.clear();
  addCurrentOdometryWaypoint();

  // Add (x,y,z) co-ordinates from file to path.
  for (size_t i = 0; i < msg->poses.size(); i++) {
    flatstate_t cwp;
    
    // ENU path co-ordinates.
    // coordinate_type_ == "enu"
      cwp.pos(0) = msg->poses[i].pose.position.x;
      cwp.pos(1) = msg->poses[i].pose.position.y;
      cwp.pos(2) = msg->poses[i].pose.position.z;
    
    initial_waypoints_.push_back(cwp);
  }

  // Add heading from file to path.
  // for (size_t i = 1; i < initial_waypoints_.size(); i++) {
  //   // heading_mode_ == "zero") {
  //     initial_waypoints_[i].yaw = initial_waypoints_[0].yaw;
    
  // }
  for (size_t i = 1; i < initial_waypoints_.size(); i++) {
    if (heading_mode_ == "manual") {
      tf::Quaternion _tmp(msg->poses[i-1].pose.orientation.x,
        msg->poses[i-1].pose.orientation.y,
        msg->poses[i-1].pose.orientation.z,
        msg->poses[i-1].pose.orientation.w);
            tf::Matrix3x3 mm(_tmp);
      double _roll, _pitch, _yaw;            
      mm.getRPY(_roll, _pitch, _yaw);
      initial_waypoints_[i].yaw=_yaw;

    } else if (heading_mode_ == "auto") {
      // Compute heading in direction towards next point.
      initial_waypoints_[i].yaw=
          atan2(initial_waypoints_[i].pos(1) -
                    initial_waypoints_[i - 1].pos(1),
                initial_waypoints_[i].pos(0) -
                    initial_waypoints_[i - 1].pos(0));
    } else if (heading_mode_ == "zero") {
      initial_waypoints_[i].yaw=0.0;
    } else {
      initial_waypoints_[i].yaw = initial_waypoints_[0].yaw;
    }
  }

  // As first target point, add current (x,y) position, but with height at
  // that of the first requested waypoint, so that the MAV first adjusts height
  // moving only vertically.
  if (initial_waypoints_.size() >= 2) {
    flatstate_t vwp;
    vwp.pos = current_odom_.pos;
    vwp.pos(2) = initial_waypoints_[1].pos(2);
    vwp.yaw = initial_waypoints_[0].yaw;     // Do not change heading.
    if ((vwp.pos-initial_waypoints_[1].pos).norm() > kIntermediatePoseTolerance)
      initial_waypoints_.insert(initial_waypoints_.begin() + 1, vwp);
  }

  // Limit maximum distance between waypoints.
  if (intermediate_poses_) {
    addIntermediateWaypoints();
  }

  int num_wp = (int)initial_waypoints_.size();
  ROS_INFO("Path loaded from file. Number of points in path: %d", num_wp);

  //conversion to matrix form for later optimisation purpose
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

  // std::cout<<"polyTime_ is "<<polyTime_<<"\n";
  // std::cout<<"polyCoeff_ is"<<polyCoeff_<<"\n";
  trajectory_obtained_ = true;

  current_leg_ = 0;
  sampleWholeTrajandVisualize();

  if (sim_type_=="dji"){
    if (trajectory_obtained_){
      start_trajectory_ = true;
      idle_state_ = false;
      ROS_INFO("Start trajectory publish!!");
    } else ROS_INFO("Something went wrong!!");
  }

  return;  
}

// void geoQuatoEigenQua (geometry_msgs::Quaternion geoQua, Eigen::Quaternion& eigenQua)
// {
//   eigenQua = Quaternion(geoQua.w, geoQua.x, geoQua.y, geoQua.z);
// }
void ReplanTimerCallback(const ros::TimerEvent&)
{
  if (got_odom_ && wall_msg_updated_ && trajectory_wip_){
    //wall_msg_updated_ = false;


    // if ((_curPos - _projectPt).length() > 0.8f) {
    //   ROS_INFO("triggering replanning..")

    //   ROS_INFO("Start solving for trajectory...");
    //   MatrixXd _updated_path(x,4)
    //   polyTime_  = timeAllocation(initial_path);
    //   polyCoeff_ = trajectoryGeneratorWaypoint.PolyQPGenerationClosedForm(dev_order_, _updated_path, vel, acc, polyTime_);
    // }

  }
}

void TimerCallback(const ros::TimerEvent&)
{
  if (got_odom_){
    if ((ros::Time::now()-current_odom_.timestamp).toSec()>0.8){
      got_odom_ = false;
    }
  }

  if (wall_msg_updated_){
    planeWorldABC_Est_ = (1-est_K_)*planeWorldABC_Est_ + est_K_*planeWorldABC_;
    planeWorldD_Est_ = (1-est_K_)*planeWorldD_Est_ + est_K_*planeWorldD_;
    // std::cout<<"planeWorldABC_ is "<<planeWorldABC_<<"\n";
    // std::cout<<"planeWorldABC_Est_ is "<<planeWorldABC_Est_<<"\n";
    
    wall_msg_updated_ = false;
  }

  geometry_msgs::PoseStamped wall_estimate_model;
  wall_estimate_model.header.stamp = ros::Time::now();
  wall_estimate_model.pose.orientation.x = planeWorldABC_Est_(0);
  wall_estimate_model.pose.orientation.y = planeWorldABC_Est_(1);
  wall_estimate_model.pose.orientation.z = planeWorldABC_Est_(2);
  wall_estimate_model.pose.orientation.w = planeWorldD_Est_;

  //get a sample at the middle of trajectory
  if (polyTime_.size()>0) {
    trajectory_msgs::MultiDOFJointTrajectory sample_point = generateTrajOnline(polyTime_.sum()/2, 0.1, 1);
    wall_estimate_model.pose.position.x = sample_point.points[0].transforms[0].translation.x;
    wall_estimate_model.pose.position.y = sample_point.points[0].transforms[0].translation.y;
    wall_estimate_model.pose.position.z = sample_point.points[0].transforms[0].translation.z;
  }
  Wall_estimate_pub.publish(wall_estimate_model);

  if (start_trajectory_){
    if (!trajectory_wip_){
      trajectory_start_time_ = ros::Time::now();
      trajectory_wip_ = true;
      //return;
    }
    double timeElasped = (ros::Time::now() - trajectory_start_time_).toSec();
    if (timeElasped > polyTime_.sum()){
      trajectory_wip_ = false;
      start_trajectory_ = false;
      idle_state_ = true;      
    }      

    trajectory_msgs::MultiDOFJointTrajectory trajset_msg = generateTrajOnline(timeElasped, 0.1, 20);  
    trajectory_pub.publish(trajset_msg);
        
        // for testing
        // //_pos = getPosPoly(polyCoeff_, nSeg_wip_, timeElasped-_timeSum);

        // nav_msgs::Odometry trajectory_odom;
        // trajectory_odom.header.stamp = ros::Time::now();
        // trajectory_odom.header.frame_id = frame_id_;
        // trajectory_odom.pose.pose.position.x = _pos(0);
        // trajectory_odom.pose.pose.position.y = _pos(1);
        // trajectory_odom.pose.pose.position.z = _pos(2);
        // trajectory_odom.pose.pose.orientation.x = 0;
        // trajectory_odom.pose.pose.orientation.y = 0;
        // trajectory_odom.pose.pose.orientation.z = sinf(_yaw*0.5);
        // trajectory_odom.pose.pose.orientation.w = cosf(_yaw*0.5);

        // trajectory_sim_pub.publish(trajectory_odom);

  } else if (idle_state_){
    if (trajectory_start_time_!= ros::Time(0.001)){
      ROS_INFO_ONCE("Just hold position then...");
      double timeElasped = (ros::Time::now() - trajectory_start_time_).toSec();
      trajectory_msgs::MultiDOFJointTrajectory trajset_msg = generateTrajOnline(timeElasped, 0.1, 20);  
      trajectory_pub.publish(trajset_msg);        
    }
  }
}

trajectory_msgs::MultiDOFJointTrajectory generateTrajOnline(double timeinTraj, double T_s, int horizon)
{
  Eigen::Vector3d _pos, _vel, _acc;
  double _yaw = 0.0;
  _pos.setZero();
  _vel.setZero();
  _acc.setZero();
  trajectory_msgs::MultiDOFJointTrajectory trajset_msg;

  for (int n = 0; n<horizon; n++){
    double _timeinTraj = timeinTraj + n*T_s;
    _pos.setZero();
    if (_timeinTraj > polyTime_.sum()){
      if (n==0) ROS_INFO("Finished publishing trajectory. Hold position instead.");

      _pos = initial_waypoints_.back().pos; //set pos and yaw as the final waypoint
      _yaw = initial_waypoints_.back().yaw;
      _vel.setZero();
      _acc.setZero();

    } else {  //find which segment it is at and publish accordingly
      int _nSeg = polyTime_.size();
      int nSeg_wip_;
      double _timeSum = 0.0;
      for (int k=0; k<_nSeg; k++){
        if (_timeinTraj < _timeSum +polyTime_[k]){
          nSeg_wip_ = k;
          break;
        } else _timeSum += polyTime_[k];
      }
      // if (n==0) {
      //   std::cout<<"now in "<<nSeg_wip_<<" number of segment, time elapsed is "<<_timeinTraj<<
      //         "time in this segment is"<< _timeinTraj-_timeSum<<"\n";        
      // }
      for (int i=0; i<3; i++){
        for (int k=0; k<2*dev_order_; k++){
          _pos(i) += polyCoeff_(nSeg_wip_, i*2*dev_order_+ k) * std::pow(_timeinTraj-_timeSum, k);
          // _vel(i) += (k>0? k*
          //             polyCoeff_(nSeg_wip_, i*2*dev_order_+ k) * std::pow(_timeinTraj-_timeSum, k-1):0);
          // _acc(i) += (k>1? k*(k-1)*
          //             polyCoeff_(nSeg_wip_, i*2*dev_order_+ k) * std::pow(_timeinTraj-_timeSum, k-2):0);
        }
      }
      for (int k=0; k<2*dev_order_; k++){
        _yaw += polyCoeff_(nSeg_wip_, 3*2*dev_order_+ k) * std::pow(_timeinTraj-_timeSum, k);
      }
      _yaw = wrapPi(_yaw);        
    }

    Eigen::Vector3d _pos_reprojected;
    // _pos_reprojected = _pos+ planeWorldABC_Est_ * (-planeWorldD_Est_ + desired_distance_s_ 
    //                                               -planeWorldABC_Est_.dot(_pos));
    Eigen::Vector3d tmp_vec(-1.0, 0.0, 0.0);

    if (sim_type_=="none_st"||sim_type_=="sim_st"){
      _pos_reprojected = _pos;
    } else if ((planeWorldABC_Est_-tmp_vec).norm()<=0.0001){ //no update in wall coefficients at all
            _pos_reprojected = _pos;

    } else{
      _pos_reprojected = _pos+ planeWorldABC_Est_ * (-planeWorldD_Est_ + desired_distance_s_ 
                                                    -planeWorldABC_Est_.dot(_pos));  
    }

     std::cout<<"pos reprojected is "<<_pos_reprojected<<"\n";
     std::cout<<"pos is "<<_pos<<"\n";
    trajectory_msgs::MultiDOFJointTrajectoryPoint trajpt_msg;
    geometry_msgs::Transform transform_msg;
    geometry_msgs::Twist accel_msg, vel_msg;
    transform_msg.translation.x = _pos_reprojected(0);
    transform_msg.translation.y = _pos_reprojected(1);
    transform_msg.translation.z = _pos_reprojected(2);
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
  }
  trajset_msg.header.stamp = ros::Time::now();
  return trajset_msg;
}

trajectory_msgs::MultiDOFJointTrajectory generateTrajNominal(double timeinTraj, double T_s, int horizon)
{
  Eigen::Vector3d _pos, _vel, _acc;
  double _yaw = 0.0;
  _pos.setZero();
  _vel.setZero();
  _acc.setZero();
  trajectory_msgs::MultiDOFJointTrajectory trajset_msg;

  for (int n = 0; n<horizon; n++){
    double _timeinTraj = timeinTraj + n*T_s;
    _pos.setZero();
    if (_timeinTraj > polyTime_.sum()){
      if (n==0) ROS_INFO("Finished publishing trajectory. Hold position instead.");

      _pos = initial_waypoints_.back().pos; //set pos and yaw as the final waypoint
      _yaw = initial_waypoints_.back().yaw;
      _vel.setZero();
      _acc.setZero();

    } else {  //find which segment it is at and publish accordingly
      int _nSeg = polyTime_.size();
      int nSeg_wip_;
      double _timeSum = 0.0;
      for (int k=0; k<_nSeg; k++){
        if (_timeinTraj < _timeSum +polyTime_[k]){
          nSeg_wip_ = k;
          break;
        } else _timeSum += polyTime_[k];
      }
      for (int i=0; i<3; i++){
        for (int k=0; k<2*dev_order_; k++){
          _pos(i) += polyCoeff_(nSeg_wip_, i*2*dev_order_+ k) * std::pow(_timeinTraj-_timeSum, k);
        }
      }
      for (int k=0; k<2*dev_order_; k++){
        _yaw += polyCoeff_(nSeg_wip_, 3*2*dev_order_+ k) * std::pow(_timeinTraj-_timeSum, k);
      }
      _yaw = wrapPi(_yaw);        
    }

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
    trajpt_msg.velocities.push_back(vel_msg);
    trajpt_msg.accelerations.push_back(accel_msg);
    trajset_msg.points.push_back(trajpt_msg);
  }
  trajset_msg.header.stamp = ros::Time::now();
  return trajset_msg;
}

bool readFileCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  // if (!got_odom_){
  //   ROS_WARN("NO ODOM, WILL NOT CALCULATE TRAJECTORY");
  //   return false;
  // }
  start_trajectory_ = false;
  idle_state_ = true;
  trajectory_wip_ = false;
  initial_waypoints_.clear();
  addCurrentOdometryWaypoint();

  // Add (x,y,z) co-ordinates from file to path.
  for (size_t i = 0; i < easting.size(); i++) {
    flatstate_t cwp;
    // GPS path co-ordinates.
    if (coordinate_type_ == "gps") {
      // double initial_latitude;
      // double initial_longitude;
      // double initial_altitude;

      // Convert GPS point to ENU co-ordinates.
      // NB: waypoint altitude = desired height above reference + registered
      // reference altitude.
      // geodetic_converter_.getReference(&initial_latitude, &initial_longitude, &initial_altitude);
      // geodetic_converter_.geodetic2Enu(northing[i], easting[i], (initial_altitude + height[i]),
      //                                 &cwp.pos(0), &cwp.pos(1), &cwp.pos(2));
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

  //conversion to matrix form for later optimisation purpose
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

  // std::cout<<"polyTime_ is "<<polyTime_<<"\n";
  // std::cout<<"polyCoeff_ is"<<polyCoeff_<<"\n";
  trajectory_obtained_ = true;

  current_leg_ = 0;
  sampleWholeTrajandVisualize();
  return true;
}

void sampleWholeTrajandVisualize()
{ 
  double sample_T = 0.5;
  double T_total = polyTime_.sum();
  
  nav_msgs::Path pathVisualizer;
  pathVisualizer.header.stamp = ros::Time::now();
  pathVisualizer.header.frame_id = frame_id_;
  for (double t=0; t<T_total; t+=sample_T){
    ros::Time t_temp(t);
    trajectory_msgs::MultiDOFJointTrajectory _traj_temp = generateTrajNominal(t, 0.1, 1);;
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.header.stamp = t_temp;
    this_pose_stamped.header.frame_id = frame_id_;
    this_pose_stamped.pose.position.x = _traj_temp.points[0].transforms[0].translation.x;
    this_pose_stamped.pose.position.y = _traj_temp.points[0].transforms[0].translation.y;
    this_pose_stamped.pose.position.z = _traj_temp.points[0].transforms[0].translation.z;
    this_pose_stamped.pose.orientation = _traj_temp.points[0].transforms[0].rotation;
    pathVisualizer.poses.push_back(this_pose_stamped);
  }
  nominal_path_publisher.publish(pathVisualizer);
}
bool executePathCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  // if (!got_odom_){
  //   ROS_WARN("NO ODOM, WILL NOT CALCULATE TRAJECTORY");
  //   return false;
  // }

  if (trajectory_obtained_){
    start_trajectory_ = true;
    idle_state_ = false;
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