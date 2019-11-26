#include <math.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Vector3.h>

#include <nav_msgs/Odometry.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <trajectory_control/DroneStates.hpp>
#include <trajectory_control/fsf.hpp>
#include <trajectory_control/kinematicTransform.hpp>
#include <trajectory_control/statesEstimator.hpp>

double G = 9.81;

struct State
{
    double r;
    double r_dot;
    double r_ddot;
    double b;
    double b_dot;
    double b_ddot;
};

/*******************************Global variables*******************************/
trajectory_msgs::JointTrajectory jointTrajectory;
DroneStates lastMeasuredStates;
geometry_msgs::Vector3 lastEulerAngles;
mavros_msgs::State drone_state;
trajectory_msgs::JointTrajectoryPoint drone_states_joint;

bool isFirstCallback = false;
/******************************************************************************/

/**********************************Functions***********************************/
void jointTrajectoryAcquireCallback(const trajectory_msgs::JointTrajectory & msg) {
  int i = 0;
  trajectory_msgs::JointTrajectoryPoint point = msg.points[0];

  //Find the index in order to update the jointTrajectory.points from the topic
  for (const auto & point_saved : jointTrajectory.points){
    if (point.time_from_start == point_saved.time_from_start) break;
    i += 1;
  }

  //Erase the values that are going to be updated
  jointTrajectory.points.erase(jointTrajectory.points.begin() + i, jointTrajectory.points.end());

  //Push the new points in jointTrajectory.points
  for (const auto & point_new : msg.points){
    jointTrajectory.points.push_back(point_new);
  }
}

void measuredStatesAcquireCallback(const nav_msgs::Odometry & msg) {

  //Update measured position and velocity from topic
  geometry_msgs::PoseWithCovariance pose = msg.pose;
  geometry_msgs::TwistWithCovariance twist = msg.twist;

  geometry_msgs::Vector3 position, velocity;

  position.x = pose.pose.position.x;
  position.y = pose.pose.position.y;
  position.z = pose.pose.position.z;

  velocity = twist.twist.linear;

  lastMeasuredStates.replacePosAndSpeed(position, velocity);

  //Update eulerAngles (euler) from topic (quaternion) -> convertion
  geometry_msgs::Quaternion q_msg;
  tf2::Quaternion q;
  tf2::Matrix3x3 m;
  geometry_msgs::Vector3 orientation;

  q_msg = pose.pose.orientation;
  tf2::convert(q_msg, q); //convert geometry_msgs::Quaternion to tf2::Quaternion
  m.setRotation(q); //compute rotation matrix from quaternion
  m.getRPY(orientation.x, orientation.y, orientation.z); //get euler angles

  lastEulerAngles = orientation;

  //Set flag for first callback
  if (!isFirstCallback) isFirstCallback = true;

  // ROS_INFO_STREAM("trajectory_control_node: measured state callback\n position:\n" << position << "orientation:\n" << lastEulerAngles << "velocity:\n" << velocity);
}

void droneStateAcquireCallback(const mavros_msgs::State::ConstPtr& msg){
    drone_state = *msg;
}

trajectory_msgs::JointTrajectoryPoint getNextTrajectoryPoint(float time){
  int i = 0;

  //Find the next trajectory point with respect to time
  for (const auto & point : jointTrajectory.points){
    if (point.time_from_start.toSec() > time) break;
    i += 1;
  }

  //Erase the outdated values
  if (i > 0) jointTrajectory.points.erase(jointTrajectory.points.begin(), jointTrajectory.points.begin() + i - 1);

  // ROS_INFO_STREAM("i: " << i << "jointTrajectory.points[0]" << jointTrajectory.points[0].positions);

  return jointTrajectory.points[0];
}

State getState(trajectory_msgs::JointTrajectoryPoint point){

  State state;

  state.r = point.positions[0];
  state.b = point.positions[1];

  state.r_dot = point.velocities[0];
  state.b_dot = point.velocities[1];

  state.r_ddot = point.accelerations[0];
  state.b_ddot = point.accelerations[1];

  return state;
}

geometry_msgs::Quaternion EulerToQuaternion(float yaw, float pitch, float roll){

  geometry_msgs::Quaternion q;
  float cy, cp, cr, sy, sp, sr;

  cy = cos(yaw * .5);
  cp = cos(pitch * .5);
  cr = cos(roll * .5);

  sy = sin(yaw * .5);
  sp = sin(pitch * .5);
  sr = sin(roll * .5);

  q.w = (cy * cp * cr) + (sy * sp * sr);
  q.x = (cy * cp * sr) - (sy * sp * cr);
  q.y = (sy * cp * sr) + (cy * sp * cr);
  q.z = (sy * cp * cr) - (cy * sp * sr);

  return q;
}
/******************************************************************************/

int main(int argc, char *argv[])
{
  /*********************************Definitions********************************/
  ros::init(argc, argv, "trajectory_control_node");
  ros::NodeHandle nh, nh_private("~");

  // Define subscribers
  ros::Subscriber jointTrajectory_sub = nh.subscribe("mavros/JointTrajectory", 10, &jointTrajectoryAcquireCallback);
  ros::Subscriber measuredStates_sub = nh.subscribe("mavros/local_position/odom", 10, &measuredStatesAcquireCallback);
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, droneStateAcquireCallback);

  // Define publishers
  ros::Publisher attitudeCmd_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  ros::Publisher estimated_state_pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("mavros/estimated_state", 10);

  // Define service client
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  /****************************************************************************/

  /************************Class & Variable Declarations***********************/
  FullStatesFeedback fsf;
  KinematicTransform kt;
  StatesEstimator se;

  DroneStates predictedStates, targetStates, measuredStates;
  trajectory_msgs::JointTrajectoryPoint trajectoryPoint;
  geometry_msgs::Vector3 eulerAngles, accelerationCmd, attitudeCmd;
  mavros_msgs::AttitudeTarget cmd;
  ros::Time time, last_request;
  float yaw, dt, time2;
  bool useStatesObserver, reset;

  double r, r_last, r_dot, r_dot_last, r_ddot, b, b_last, b_dot, u_r, u_b, u_t0, delta, delta_dot;
  double m_m, m_d, wn_r, wn_b, t0;
  double z_r, z_b, a_r, a_b, b_r, b_b, v_r, v_b;
  double thrust, phi;
  State state_c;

  reset = true;
  /****************************************************************************/

  /*********************************Parameters*********************************/
  // Rate of the controller
  ros::Rate rate = nh_private.param<int>("rate", 100);

  // Récuperation des parametres  m_m, m_d, w_nr, w_nb, t_0;
  nh_private.param("m_m", m_m, 0.11);
  nh_private.param("m_d", m_d, 0.0481);
  nh_private.param("wn_r", wn_r, 0.01);
  nh_private.param("t0", t0, 1.);

  wn_b = wn_r / 2;

  r = .0;
  r_last = .0;
  r_dot = .0;
  r_dot_last = .0;
  r_ddot = .0;
  b = .0;
  b_last = .0;
  b_dot = .0;
  u_r = .0;
  u_b = .0;
  u_t0 = .0;

  // // Kinematic Transform Parameters (= physical parameters)
  // nh_private.param("mass", kt.param.mass, (float) 1.);
  // nh_private.param("hoverCompensation", kt.param.hoverCompensation, (float) .3);
  // nh_private.param("maxAngle", kt.param.maxAngle, (float) 45.);
  // nh_private.param("maxVerticalAcceleration", kt.param.maxVerticalAcceleration, (float) 4.);
  //
  // ROS_INFO_STREAM(
  //   "\n********* System Parameters (can be defined in launchfile) *********"
  //   << "\nfsf_x_P: " << fsf.x.param.Kp << " fsf_x_S: " << fsf.x.param.Ks
  //   << "\nfsf_y_P: " << fsf.y.param.Kp << " fsf_y_S: " << fsf.y.param.Ks
  //   << "\nfsf_z_P: " << fsf.z.param.Kp << " fsf_z_S: " << fsf.z.param.Ks
  //   << "\n"
  //   << "\nuse_StatesObserver: " << useStatesObserver
  //   << "\nse_x_Lpos: " << se.x.param.Lpos << " nse_x_Lspeed: " << se.x.param.Lspeed << " se_x_Lunc: " << se.x.param.Lunc
  //   << "\nse_y_Lpos: " << se.y.param.Lpos << " nse_y_Lspeed: " << se.y.param.Lspeed << " se_y_Lunc: " << se.y.param.Lunc
  //   << "\nse_z_Lpos: " << se.z.param.Lpos << " nse_z_Lspeed: " << se.z.param.Lspeed << " se_z_Lunc: " << se.z.param.Lunc
  //   << "\n"
  //   << "\nse_x_Filter: " << se.x.param.filterCoeff
  //   << "\nse_y_Filter: " << se.y.param.filterCoeff
  //   << "\nse_z_Filter: " << se.z.param.filterCoeff
  //   << "\n"
  //   << "\nmass: " << kt.param.mass
  //   << "\nhoverCompensation: " << kt.param.hoverCompensation
  //   << "\nmaxAngle: " << kt.param.maxAngle
  //   << "\nmaxVerticalAcceleration: " << kt.param.maxVerticalAcceleration
  //   << "\n*******************************************************************"
  // );
  /****************************************************************************/

  /****************************Connection & Callback***************************/
  // Wait for the drone to connect
  ROS_INFO("Waiting for drone connection ...");
  while(ros::ok() && !drone_state.connected){
      ros::spinOnce();
      rate.sleep();
  }

  // Wait for the first state measure callback
  ROS_INFO("Waiting for position measurement callback ...");
  while (ros::ok() && !isFirstCallback){
    ros::spinOnce();
    rate.sleep();
  }

  // Wait for better measure accuracy (GPS/MOCAP) -> sleep for 2 seconds
  ros::Duration(2.).sleep();
  /****************************************************************************/

  /*******************************Initialization*******************************/
  // Initialize first trajectory point as the measured position
  trajectory_msgs::JointTrajectoryPoint firstPoint;

  geometry_msgs::Vector3 position = lastMeasuredStates.getVectPos();
  geometry_msgs::Vector3 velocity = lastMeasuredStates.getVectSpeed();

  firstPoint.positions.push_back(0.2);
  firstPoint.positions.push_back(M_PI/2);
  firstPoint.velocities.push_back(0.);
  firstPoint.velocities.push_back(0.);
  firstPoint.accelerations.push_back(0.);
  firstPoint.accelerations.push_back(0.);
  firstPoint.time_from_start = ros::Duration(.0);

  jointTrajectory.points.push_back(firstPoint);

  // Set OFFBOARD mode request
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  // Set arming request
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  // Initialize time & last_request
  ros::Duration(rate.expectedCycleTime()).sleep();
  time = ros::Time::now() - rate.expectedCycleTime();
  last_request = ros::Time::now();
  /****************************************************************************/
  int n = 0;
  while (ros::ok()) {

    /****************************Manage Drone Mode*****************************/
    if( drone_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
      if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) ROS_INFO("offb_node: Offboard enabled");
      last_request = ros::Time::now();
    } else {
      if( !drone_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
        if( arming_client.call(arm_cmd) && arm_cmd.response.success) ROS_INFO("offb_node: Vehicle armed");
        last_request = ros::Time::now();
      }
    }
    /**************************************************************************/

    /*******************************Update Loop********************************/
    // Compute dt and save time
    dt = (ros::Time::now().toNSec() - time.toNSec())/1000000000.0f;
    time = ros::Time::now();

    // Consider time for trajectory only when the drone is armed & set reset flag to true
    if (drone_state.armed) time2 += dt;
    else reset = true;

    // Get the next trajectory point and update the target state
    trajectoryPoint = getNextTrajectoryPoint(time2);
    state_c = getState(trajectoryPoint);
    /**************************************************************************/

    /*************************Linear Feedback Control**************************/
    delta = atan2(position.y, position.x);
    delta_dot = (velocity.y * position.x - position.y * velocity.x) / (position.x * position.x + position.y * position.y);

    r = sqrt(position.x*position.x + position.y*position.y + position.z*position.z);
    // r_dot = (r - r_last) / dt;
    // r_ddot = (r_dot - r_dot_last) / dt;
    r_dot = (position.x + position.y + position.z) / r;
    r_ddot = ((velocity.x + velocity.y + velocity.z) * r - (position.x + position.y + position.z) * r_dot) / (r * r);

    b = atan2(position.z, position.x/cos(delta)); // if (delta == pi/2)
    // b_dot = (b - b_last) / dt;
    b_dot = (cos(delta) * velocity.z * position.x - position.z * (velocity.x * cos(delta) + position.x * delta_dot * sin(delta))) / (position.x * position.x + position.z * position.z * cos(delta) * cos(delta));

    r_last = r;
    b_last = b;

    r_dot_last = r_dot;
    ROS_INFO_STREAM("x: " << position.x << " y: " << position.y << " z: " << position.z);
    ROS_INFO_STREAM("r: " << r << " b: " << b << " d: " << delta);
    ROS_INFO_STREAM("r_dot: " << r_dot << " r_ddot: " << r_ddot);
    ROS_INFO_STREAM("b_dot: " << b_dot);
    ROS_INFO_STREAM("state_c.r: " << state_c.r << " state_c.b: " << state_c.b << " state_c.r_dot: " << state_c.r_dot << " state_c.b_dot: " << state_c.b_dot);

    z_r += (state_c.r - r) * dt;
    z_b += (state_c.b - b) * dt;

    ROS_INFO_STREAM("t0: " << t0 << " m_m: " << m_m << " G: " << G << " sin(b): " << sin(b) << " u_r: " << u_r << " m_d: " << m_d);

    u_t0 = t0 + m_m * r_ddot - r * b_dot * b_dot * m_m + m_m * G * sin(b) + t0 - u_r - m_d * r_ddot;
    // u_t0 = 2 * t0;

    a_r = - (m_m + m_d);
    a_b = m_m * r;

    b_r = (r * b_dot * b_dot * m_m - m_m * G * sin(b) + u_t0) / (m_m + m_d);
    b_b = (-2 * b_dot * r_dot - m_m * G / m_m * cos(b)) / r;

    v_r = 2.15 * wn_r * wn_r * (state_c.r - r) + wn_r * wn_r * wn_r * z_r + state_c.r_ddot + 1.75 * wn_r * (state_c.r_dot - r_dot);
    v_b = 2.15 * wn_b * wn_b * (state_c.b - b) + wn_b * wn_b * wn_b * z_b + state_c.b_ddot + 1.75 * wn_b * (state_c.b_dot - b_dot);

    u_r = a_r * (v_r - b_r);
    u_b = a_b * (v_b - b_b);

    ROS_INFO_STREAM("u_t0: " << u_t0 << " u_r: " << u_r << " u_b: " << u_b);

    thrust = sqrt(u_t0 * u_t0 + u_b * u_b);
    phi = atan2(-u_t0, u_b) + b;
    /**************************************************************************/

    /*************************Publish Attitude Command*************************/
    ROS_INFO_STREAM("phi: " << phi << " thrust: " << thrust);
    cmd.orientation = EulerToQuaternion(.0, phi, .0);
    cmd.body_rate = geometry_msgs::Vector3();
    cmd.thrust = thrust;
    cmd.type_mask = 7; // ignore body rate

    // For Testing:
    // cmd.orientation = EulerToQuaternion(0, 0.5, 0); // (yaw, pitch, roll)
    // cmd.body_rate = geometry_msgs::Vector3();
    // cmd.thrust = 0.7;
    // cmd.type_mask = 7; // ignore body rate

    attitudeCmd_pub.publish(cmd);
    estimated_state_pub.publish(drone_states_joint);
    /**************************************************************************/

    /***************************Publish Pose Command***************************/
    // For Testing:
    // geometry_msgs::PoseStamped pose;
    // pose.pose.position.x = 0;
    // pose.pose.position.y = 0;
    // pose.pose.position.z = 0;
    //
    // local_pos_pub.publish(pose);
    /**************************************************************************/

    n += 1;
    // if (n > 10) return 0;

    ros::spinOnce();
    rate.sleep();
  }
	return 0;
}
