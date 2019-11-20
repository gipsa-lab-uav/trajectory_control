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

DroneStates getState(trajectory_msgs::JointTrajectoryPoint point){

  DroneStates state;
  geometry_msgs::Vector3 position, velocity, acceleration;

  position.x = point.positions[0];
  position.y = point.positions[1];
  position.z = point.positions[2];

  velocity.x = point.velocities[0];
  velocity.y = point.velocities[1];
  velocity.z = point.velocities[2];

  acceleration.x = point.accelerations[0];
  acceleration.y = point.accelerations[1];
  acceleration.z = point.accelerations[2];

  state.fillStates(position, velocity, acceleration);

  return state;
}

trajectory_msgs::JointTrajectoryPoint getJointTrajectoryPoint(DroneStates state){

  trajectory_msgs::JointTrajectoryPoint point;
  geometry_msgs::Vector3 position, velocity, acceleration;
  position = state.getVectPos();
  velocity= state.getVectSpeed();
  acceleration = state.getVectAcceleration();

  point.positions.push_back(position.x);
  point.positions.push_back(position.y);
  point.positions.push_back(position.z);

  point.velocities.push_back(velocity.x);
  point.velocities.push_back(velocity.y);
  point.velocities.push_back(velocity.z);

  point.accelerations.push_back(acceleration.x);
  point.accelerations.push_back(acceleration.y);
  point.accelerations.push_back(acceleration.z);

  return point;
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

  reset = true;
  /****************************************************************************/

  /*********************************Parameters*********************************/
  // Rate of the controller
  ros::Rate rate = nh_private.param<int>("rate", 100);

  // Full States Feedback Gains
  nh_private.param("fsf_x_P", fsf.x.param.Kp, (float) 1.0);
  nh_private.param("fsf_x_S", fsf.x.param.Ks, (float) 1.7);
  nh_private.param("fsf_y_P", fsf.y.param.Kp, (float) 1.0);
  nh_private.param("fsf_y_S", fsf.y.param.Ks, (float) 1.7);
  nh_private.param("fsf_z_P", fsf.z.param.Kp, (float) 1.56);
  nh_private.param("fsf_z_S", fsf.z.param.Ks, (float) 2.4);

  // States Observer Gains
  nh_private.param("use_StatesObserver", useStatesObserver, true);

  nh_private.param("se_x_Lpos", se.x.param.Lpos, (float) 1.03);
  nh_private.param("se_y_Lpos", se.y.param.Lpos, (float) 1.03);
  nh_private.param("se_z_Lpos", se.z.param.Lpos, (float) 1.03);
  nh_private.param("se_x_Lspeed", se.x.param.Lspeed, (float) 10.7);
  nh_private.param("se_y_Lspeed", se.y.param.Lspeed, (float) 10.7);
  nh_private.param("se_z_Lspeed", se.z.param.Lspeed, (float) 10.7);
  nh_private.param("se_x_Lunc", se.x.param.Lunc, (float) 9.62);
  nh_private.param("se_y_Lunc", se.y.param.Lunc, (float) 9.62);
  nh_private.param("se_z_Lunc", se.z.param.Lunc, (float) 9.62);

  // Actuators Time Constant (used to filter the command fed to the observer)
  nh_private.param("se_x_Filter", se.x.param.filterCoeff, (float) .9);
  nh_private.param("se_y_Filter", se.y.param.filterCoeff, (float) .9);
  nh_private.param("se_z_Filter", se.z.param.filterCoeff, (float) .15);

  // Kinematic Transform Parameters (= physical parameters)
  nh_private.param("mass", kt.param.mass, (float) 1.);
  nh_private.param("hoverCompensation", kt.param.hoverCompensation, (float) .3);
  nh_private.param("maxAngle", kt.param.maxAngle, (float) 45.);
  nh_private.param("maxVerticalAcceleration", kt.param.maxVerticalAcceleration, (float) 4.);

  ROS_INFO_STREAM(
    "\n********* System Parameters (can be defined in launchfile) *********"
    << "\nfsf_x_P: " << fsf.x.param.Kp << " fsf_x_S: " << fsf.x.param.Ks
    << "\nfsf_y_P: " << fsf.y.param.Kp << " fsf_y_S: " << fsf.y.param.Ks
    << "\nfsf_z_P: " << fsf.z.param.Kp << " fsf_z_S: " << fsf.z.param.Ks
    << "\n"
    << "\nuse_StatesObserver: " << useStatesObserver
    << "\nse_x_Lpos: " << se.x.param.Lpos << " nse_x_Lspeed: " << se.x.param.Lspeed << " se_x_Lunc: " << se.x.param.Lunc
    << "\nse_y_Lpos: " << se.y.param.Lpos << " nse_y_Lspeed: " << se.y.param.Lspeed << " se_y_Lunc: " << se.y.param.Lunc
    << "\nse_z_Lpos: " << se.z.param.Lpos << " nse_z_Lspeed: " << se.z.param.Lspeed << " se_z_Lunc: " << se.z.param.Lunc
    << "\n"
    << "\nse_x_Filter: " << se.x.param.filterCoeff
    << "\nse_y_Filter: " << se.y.param.filterCoeff
    << "\nse_z_Filter: " << se.z.param.filterCoeff
    << "\n"
    << "\nmass: " << kt.param.mass
    << "\nhoverCompensation: " << kt.param.hoverCompensation
    << "\nmaxAngle: " << kt.param.maxAngle
    << "\nmaxVerticalAcceleration: " << kt.param.maxVerticalAcceleration
    << "\n*******************************************************************"
  );
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
  // Initialize acceleration command
  accelerationCmd.x = .0;
  accelerationCmd.y = .0;
  accelerationCmd.z = .0;

  // Initialize first trajectory point as the measured position
  trajectory_msgs::JointTrajectoryPoint firstPoint;

  geometry_msgs::Vector3 position = lastMeasuredStates.getVectPos();

  firstPoint.positions.push_back(position.x);
  firstPoint.positions.push_back(position.y);
  firstPoint.positions.push_back(position.z - .01f);
  firstPoint.positions.push_back(lastEulerAngles.z);
  firstPoint.velocities.push_back(0.);
  firstPoint.velocities.push_back(0.);
  firstPoint.velocities.push_back(0.);
  firstPoint.accelerations.push_back(0.);
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
    targetStates = getState(trajectoryPoint);

    // Get the yaw from the trajectoryPoint
    yaw = trajectoryPoint.positions[3];

    // Get the last measured state
    measuredStates = lastMeasuredStates;
    eulerAngles = lastEulerAngles;
    /**************************************************************************/

    /*************************Full State Feedback & KT*************************/
    // Reset estimator and set reset flag to false
    if (drone_state.armed && reset){
      se.resetEstimations();
      reset = false;
    }

    // Get the estimated state from measures and previous estimation & command
    if(!useStatesObserver) {
      predictedStates = se.process(dt, measuredStates.getVectPos(), predictedStates, accelerationCmd);
      measuredStates.x.uncertainties = predictedStates.x.uncertainties;
      measuredStates.y.uncertainties = predictedStates.y.uncertainties;
      measuredStates.z.uncertainties = predictedStates.z.uncertainties;
      predictedStates = measuredStates;
    } else predictedStates = se.process(dt, measuredStates.getVectPos(), predictedStates, accelerationCmd);

    drone_states_joint = getJointTrajectoryPoint(predictedStates);
    // Compute full state feedback control
    accelerationCmd = fsf.process(dt, predictedStates, targetStates);
    // ROS_INFO_STREAM("\ntime: " << time << "\ndt: " << dt << "\n\npredictedStates:\n" << predictedStates.getVectPos() << "\ntargetStates:\n" << targetStates.getVectPos() << "\nattitudeCmd:\n" << attitudeCmd);
    ROS_INFO_STREAM("z.uncertainties: " << predictedStates.z.uncertainties);
    // if (attitudeCmd.z > 0.1) break;

    // Generate (roll, pitch, thrust) command
    // For compatibility with different aircrafts or even terrestrial robots, this should be in its own node
    attitudeCmd = kt.process(accelerationCmd, eulerAngles);
    /**************************************************************************/

    /*************************Publish Attitude Command*************************/
    // Convert command in geometry_msgs::Vector3 to geometry_msgs::AttitudeTarget
    cmd.orientation = EulerToQuaternion(yaw, attitudeCmd.y, attitudeCmd.x);
    cmd.body_rate = geometry_msgs::Vector3();
    cmd.thrust = attitudeCmd.z;
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

    ros::spinOnce();
    rate.sleep();
  }
	return 0;
}
