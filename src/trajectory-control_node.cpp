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

#include <trajectory-control/DroneStates.hpp>
#include <trajectory-control/fsf.hpp>
#include <trajectory-control/kinematicTransform.hpp>
#include <trajectory-control/statesEstimator.hpp>

trajectory_msgs::JointTrajectory jointTrajectory, jointTrajectorySaved;
DroneStates lastMeasuredStates;
geometry_msgs::Vector3 lastEulerAngles;
mavros_msgs::State drone_state;

bool isFirstCallback = false;

void jointTrajectoryAcquireCallback(const trajectory_msgs::JointTrajectory & msg) {
  ROS_INFO("trajectory_control_node: jointTrajectory callback");

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
  ROS_INFO_STREAM("getNextTrajectoryPoint at time2: " << time);

  int i = 0;

  //Find the next trajectory point with respect to time
  for (const auto & point : jointTrajectory.points){
    if (point.time_from_start.toSec() > time) break;
    i += 1;
  }

  //Erase the outdated values
  if (i > 0) jointTrajectory.points.erase(jointTrajectory.points.begin(), jointTrajectory.points.begin() + i - 1);
  // jointTrajectory.points.erase(jointTrajectory.points.begin(), jointTrajectory.points.begin() + i);

  ROS_INFO_STREAM("i: " << i << "jointTrajectory.points[0]" << jointTrajectory.points[0]);

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

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "trajectory_control_node");
  ros::NodeHandle nh, nh_private("~");

  // Define subscribers
  ros::Subscriber jointTrajectory_sub = nh.subscribe("mavros/JointTrajectory", 10, &jointTrajectoryAcquireCallback);
  ros::Subscriber measuredStates_sub = nh.subscribe("mavros/local_position/odom", 10, &measuredStatesAcquireCallback);
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, droneStateAcquireCallback);


  // Define publishers
  ros::Publisher attitudeCmd_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

  // Define service client
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  // Declare classes & variables
  FullStatesFeedback fsf;
  KinematicTransform kt;
  StatesEstimator se;

  DroneStates predictedStates, targetStates, measuredStates;
  trajectory_msgs::JointTrajectoryPoint trajectoryPoint;
  geometry_msgs::Vector3 eulerAngles, accelerationCmd, attitudeCmd;
  mavros_msgs::AttitudeTarget cmd;
  ros::Time time, time2, last_request;
  float yaw, dt;

  accelerationCmd.x = .0;
  accelerationCmd.y = .0;
  accelerationCmd.z = .0;

  /** PARAMETERS **/
  // Full States Feedback Gains
  fsf.x.param.Kp = nh_private.param<double>("fsf_x_P", 0.3f);
  fsf.x.param.Ks = nh_private.param<double>("fsf_x_S", 2.5f);
  fsf.y.param.Kp = nh_private.param<double>("fsf_y_P", 0.3f);
  fsf.y.param.Ks = nh_private.param<double>("fsf_y_S", 2.5f);
  fsf.z.param.Kp = nh_private.param<double>("fsf_z_P", 0.3f);
  fsf.z.param.Ks = nh_private.param<double>("fsf_z_S", 2.5f);

  // States Observer Gains
  bool useStatesObserver = nh_private.param<bool>("use_StatesObserver", true);
  se.x.param.Lpos = nh_private.param<double>("se_x_Lpos", 1.03f);
  se.x.param.Lspeed = nh_private.param<double>("se_x_Lspeed", 10.7f);
  se.x.param.Lunc = nh_private.param<double>("se_x_Lunc", 9.62f);
  se.y.param.Lpos = nh_private.param<double>("se_y_Lpos", 1.03f);
  se.y.param.Lspeed = nh_private.param<double>("se_y_Lspeed", 10.7f);
  se.y.param.Lunc = nh_private.param<double>("se_y_Lunc", 9.62f);
  se.z.param.Lpos = nh_private.param<double>("se_z_Lpos", 1.03f);
  se.z.param.Lspeed = nh_private.param<double>("se_z_Lspeed", 10.7f);
  se.z.param.Lunc = nh_private.param<double>("se_z_Lunc", 9.62f);

  // Actuators Time Constant (used to filter the command fed to the observer)
  se.x.param.filterCoeff = nh_private.param<double>("se_x_Filter", 0.95f);
  se.y.param.filterCoeff = nh_private.param<double>("se_y_Filter", 0.95f);
  se.z.param.filterCoeff = nh_private.param<double>("se_z_Filter", 0.15f);

  // Kinematic Transform Parameters (= physical parameters)
  kt.param.hoverCompensation = nh_private.param<double>("hoverCompensation",0.45f);
  kt.param.mass = nh_private.param<double>("mass",1.5f);
  kt.param.maxAngle = nh_private.param<double>("maxAngle",45.0f);
  kt.param.maxVerticalAcceleration = nh_private.param<double>("maxVerticalAcceleration",4.0f);

  // Rate of the controller
  ros::Rate rate = nh_private.param<int>("rate", 100);

  /** END PARAMETERS **/

  //Wait for the drone to connect
  while(ros::ok() && !drone_state.connected){
      ros::spinOnce();
      rate.sleep();
  }

  //Wait for the first state measure callback
  while (ros::ok() && !isFirstCallback){
    ros::spinOnce();
    rate.sleep();
  }

  //Wait for better measure accuracy (GPS/MOCAP) -> sleep for 2 seconds
  ros::Duration(2.).sleep();

  //Initialize first trajectory point as the measured position
  trajectory_msgs::JointTrajectoryPoint firstPoint;

  geometry_msgs::Vector3 position = lastMeasuredStates.getVectPos();

  firstPoint.positions.push_back(position.x);
  firstPoint.positions.push_back(position.y);
  firstPoint.positions.push_back(position.z);
  firstPoint.velocities.push_back(0.);
  firstPoint.velocities.push_back(0.);
  firstPoint.velocities.push_back(0.);
  firstPoint.accelerations.push_back(0.);
  firstPoint.accelerations.push_back(0.);
  firstPoint.accelerations.push_back(0.);
  firstPoint.time_from_start = ros::Duration(.0);

  jointTrajectory.points.push_back(firstPoint);

  //Request OFFBOARD mode
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  //Request arming
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  last_request = ros::Time::now();

  //Initialize time
  ros::Duration(rate.expectedCycleTime()).sleep();
  time = ros::Time::now() - rate.expectedCycleTime();

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
    //Consider time for trajectory only when the drone is armed
    if ( drone_state.armed ) time2 += ros::Time::now() - time;

    //Compute dt and save time
    dt = (ros::Time::now().toNSec() - time.toNSec())/1000000000.0f;
    time = ros::Time::now();

    //Get the next trajectory point and update the target state
    trajectoryPoint = getNextTrajectoryPoint(time2.toSec());
    targetStates = getState(trajectoryPoint);

    //Save the next trajectory point to jointTrajectorySaved
    jointTrajectorySaved.points.push_back(trajectoryPoint);

    //Get the yaw from the trajectoryPoint
    yaw = trajectoryPoint.positions[3];

    //Get the last measured state
    measuredStates = lastMeasuredStates;
    eulerAngles = lastEulerAngles;
    /**************************************************************************/

    /*************************Full State Feedback & KT*************************/
    //Get the estimated state from measures and previous estimation & command
    if(!useStatesObserver) {
      predictedStates = se.process(dt, measuredStates.getVectPos(), measuredStates, accelerationCmd);
      measuredStates.x.uncertainties = predictedStates.x.uncertainties;
      measuredStates.y.uncertainties = predictedStates.y.uncertainties;
      measuredStates.z.uncertainties = predictedStates.z.uncertainties;
      predictedStates = measuredStates;
    } else predictedStates = se.process(dt, measuredStates.getVectPos(), predictedStates, accelerationCmd);

    //Compute full state feedback control
    accelerationCmd = fsf.process(dt, predictedStates, targetStates);

    //Generate (roll, pitch, thrust) command
    //For compatibility with different aircrafts or even terrestrial robots, this should be in its own node
    attitudeCmd = kt.process(accelerationCmd, eulerAngles);
    /**************************************************************************/

    /*************************Publish Attitude Command*************************/
    //Convert command in geometry_msgs::Vector3 to geometry_msgs::AttitudeTarget
    cmd.orientation = EulerToQuaternion(yaw, attitudeCmd.y, attitudeCmd.x);
    cmd.body_rate = geometry_msgs::Vector3();
    cmd.thrust = attitudeCmd.z;
    cmd.type_mask = 7; // ignore body rate

    //For Testing:
    // cmd.orientation = EulerToQuaternion(0, 0.5, 0); // (yaw, pitch, roll)
    // cmd.body_rate = geometry_msgs::Vector3();
    // cmd.thrust = 0.7;
    // cmd.type_mask = 7; // ignore body rate

    attitudeCmd_pub.publish(cmd);
    /**************************************************************************/

    /***************************Publish Pose Command***************************/
    //For Testing:
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
