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
std::string trajectoryID;
trajectory_msgs::JointTrajectory jointTrajectory;
DroneStates lastMeasuredStates;
geometry_msgs::Vector3 lastEulerAngles;
mavros_msgs::State droneState;

bool isMeasuredStatesFirstCallback = false;
bool isjointTrajectoryFirstCallback = false;

/******************************************************************************/

/**********************************Functions***********************************/
void jointTrajectoryAcquireCallback(const trajectory_msgs::JointTrajectoryConstPtr &msg)
{
	if ((!isjointTrajectoryFirstCallback) && (msg->header.stamp + msg->points.back().time_from_start > ros::Time::now()))
  {
    isjointTrajectoryFirstCallback = true;
  }

	jointTrajectory = *msg;
}

void measuredStatesAcquireCallback(const nav_msgs::OdometryConstPtr &msg)
{
  // Update measured position (expressed in inertial frame) and velocity (in body frame) from topic
  geometry_msgs::Vector3 position, velocity, orientation;
  tf2::Quaternion q;
  tf2::Vector3 tf2_velocity;
  tf2::Matrix3x3 m;

  position.x = msg->pose.pose.position.x;
  position.y = msg->pose.pose.position.y;
  position.z = msg->pose.pose.position.z;

  tf2::fromMsg(msg->pose.pose.orientation, q);
  tf2::fromMsg(msg->twist.twist.linear, tf2_velocity);

  tf2_velocity = tf2::quatRotate(q, tf2_velocity);

  velocity.x = tf2_velocity.x();
  velocity.y = tf2_velocity.y();
  velocity.z = tf2_velocity.z();

  lastMeasuredStates.replacePosAndSpeed(position, velocity);

  m.setRotation(q);
  m.getRPY(orientation.x, orientation.y, orientation.z);

  lastEulerAngles = orientation;

  // Set flag for first callback
  if (!isMeasuredStatesFirstCallback)
    isMeasuredStatesFirstCallback = true;
}

void droneStateAcquireCallback(const mavros_msgs::State::ConstPtr &msg)
{
  droneState = *msg;
}

trajectory_msgs::JointTrajectoryPoint getNextTrajectoryPoint(const double &time)
{
  int i = 0;
	double trajectoryTime = jointTrajectory.header.stamp.toSec();
	
  // Find the next trajectory point with respect to time
  for (const auto &point : jointTrajectory.points)
  {
    if (trajectoryTime + point.time_from_start.toSec() > time) break;
    i += 1;
  }

  // Erase the outdated values
  if (i > 0) jointTrajectory.points.erase(jointTrajectory.points.begin(), jointTrajectory.points.begin() + i - 1);

  return jointTrajectory.points[0];
}

DroneStates getState(trajectory_msgs::JointTrajectoryPoint point)
{

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

trajectory_msgs::JointTrajectoryPoint getJointTrajectoryPoint(DroneStates state)
{

  trajectory_msgs::JointTrajectoryPoint point;
  geometry_msgs::Vector3 position, velocity, acceleration;

  position = state.getVectPos();
  velocity = state.getVectSpeed();
  acceleration = state.getVectAcceleration();

  point.positions.push_back(position.x);
  point.positions.push_back(position.y);
  point.positions.push_back(position.z);
  point.positions.push_back(state.heading);

  point.velocities.push_back(velocity.x);
  point.velocities.push_back(velocity.y);
  point.velocities.push_back(velocity.z);

  point.accelerations.push_back(acceleration.x);
  point.accelerations.push_back(acceleration.y);
  point.accelerations.push_back(acceleration.z);

  return point;
}

DroneStates getLastMeasuredStates() { return lastMeasuredStates; }

geometry_msgs::Vector3 getLastEulerAngles() { return lastEulerAngles; }

geometry_msgs::Quaternion EulerToQuaternion(float yaw, float pitch, float roll)
{

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
  ros::Publisher estimatedStates_pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("mavros/statesEstimated", 10);
  ros::Publisher referenceStates_pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("mavros/statesReference", 10);
  ros::Publisher ekfStates_pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("mavros/statesEKF", 10);
  ros::Publisher accelerationCmd_pub = nh.advertise<geometry_msgs::Vector3>("mavros/accelerationCmd", 10);
  ros::Publisher uncertainties_pub = nh.advertise<geometry_msgs::Vector3>("mavros/statesEstimatedUncertainties", 10);

  // Define service client
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  /****************************************************************************/

  /************************Class & Variable Declarations***********************/
  FullStatesFeedback fsf;
  KinematicTransform kt;
  StatesEstimator se, seAcc;

  DroneStates predictedStates, predictedStatesAcc, targetStates, targetStatesNext, measuredStates, estimatedStates, previousMeasuredStates;
  trajectory_msgs::JointTrajectoryPoint trajectoryPoint, trajectoryPointNext;
  trajectory_msgs::JointTrajectoryPoint estimatedStatesTrajectoryPoint, targetStatesTrajectoryPoint, ekfStatesTrajectoryPoint;
  geometry_msgs::Vector3 eulerAngles, accelerationCmd, attitudeCmd, position, emptyCmd;
  mavros_msgs::AttitudeTarget cmd;
  ros::Time time, last_request;
  float yaw, dt, filterPercent;
  int ctrl_freq;
  bool simulated_env, useStatesObserver, reset;

  reset = true;
  /****************************************************************************/

  /*********************************Parameters*********************************/
  // Simulated environment
  nh_private.param("simulated_env", simulated_env, true);

  // Rate of the controller
  nh_private.param("ctrl_freq", ctrl_freq, 100);

  // Full States Feedback Gains
  nh_private.param("fsf_FF", fsf.useFeedForward, false);
  nh_private.param("fsf_INT", fsf.useIntegrator, false);
  nh_private.param("fsf_x_I", fsf.x.param.Ki, (float)0.0);
  nh_private.param("fsf_x_P", fsf.x.param.Kp, (float)1.0);
  nh_private.param("fsf_x_S", fsf.x.param.Ks, (float)1.7);
  nh_private.param("fsf_y_I", fsf.y.param.Ki, (float)0.0);
  nh_private.param("fsf_y_P", fsf.y.param.Kp, (float)1.0);
  nh_private.param("fsf_y_S", fsf.y.param.Ks, (float)1.7);
  nh_private.param("fsf_z_I", fsf.z.param.Ki, (float)0.0);
  nh_private.param("fsf_z_P", fsf.z.param.Kp, (float)1.56);
  nh_private.param("fsf_z_S", fsf.z.param.Ks, (float)2.4);

  // States Observer Gains
  nh_private.param("use_StatesObserver", useStatesObserver, true);

  nh_private.param("se_x_Lpos", se.x.param.Lpos, (float)1.03);
  nh_private.param("se_y_Lpos", se.y.param.Lpos, (float)1.03);
  nh_private.param("se_z_Lpos", se.z.param.Lpos, (float)1.03);
  nh_private.param("se_x_Lspeed", se.x.param.Lspeed, (float)10.7);
  nh_private.param("se_y_Lspeed", se.y.param.Lspeed, (float)10.7);
  nh_private.param("se_z_Lspeed", se.z.param.Lspeed, (float)10.7);
  nh_private.param("se_x_Lunc", se.x.param.Lunc, (float)9.62);
  nh_private.param("se_y_Lunc", se.y.param.Lunc, (float)9.62);
  nh_private.param("se_z_Lunc", se.z.param.Lunc, (float)9.62);

  // Actuators Time Constant (used to filter the command fed to the observer)
  nh_private.param("se_x_Filter", se.x.param.filterCoeff, (float).9);
  nh_private.param("se_y_Filter", se.y.param.filterCoeff, (float).9);
  nh_private.param("se_z_Filter", se.z.param.filterCoeff, (float).15);
  seAcc.x.param = se.x.param;
  seAcc.y.param = se.y.param;
  seAcc.z.param = se.z.param;

  // Kinematic Transform Parameters (= physical parameters)
  nh_private.param("mass", kt.param.mass, (float)1.);
  nh_private.param("hoverCompensation", kt.param.hoverCompensation, (float).3);
  nh_private.param("maxAngle", kt.param.maxAngle, (float)45.);
  nh_private.param("maxVerticalAcceleration", kt.param.maxVerticalAcceleration, (float)4.);
  nh_private.param("Ky", kt.param.Ky, (float).4);
  nh_private.param("compensateYaw", kt.param.compensateYaw, true);

  ROS_INFO_STREAM(
      "\n********* System Parameters (can be defined in launchfile) *********"
      << "\nsimulated_env: " << simulated_env
      << "\n"
      << "\nctrl_freq: " << ctrl_freq
      << "\n"
      << "\nfsf_x_P: " << fsf.x.param.Kp << " fsf_x_S: " << fsf.x.param.Ks
      << "\nfsf_y_P: " << fsf.y.param.Kp << " fsf_y_S: " << fsf.y.param.Ks
      << "\nfsf_z_P: " << fsf.z.param.Kp << " fsf_z_S: " << fsf.z.param.Ks
      << "\n"
      << "\nuse_StatesObserver: " << useStatesObserver
      << "\n"
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
      << "\ncompensateYaw: " << kt.param.compensateYaw
      << "\nKy: " << kt.param.Ky
      << "\n*******************************************************************");

  ros::Rate rate = nh_private.param<int>("rate", ctrl_freq);
  /****************************************************************************/

  /****************************Connection & Callback***************************/
  // Wait for the drone to connect
  ROS_INFO("Waiting for drone connection ...");
  while (ros::ok() && !droneState.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("Drone connected.");

  // Wait for the first state measure callback
  ROS_INFO("Waiting for position measurement callback ...");
  while (ros::ok() && !isMeasuredStatesFirstCallback)
  {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("Position measurement callback ok.");

  // Wait for better measure accuracy (GPS/MOCAP) -> sleep for 2 seconds
  ros::Duration(2.).sleep();
  /****************************************************************************/

  /*******************************Initialization*******************************/
  filterPercent = 0.0f;

  emptyCmd.x = 0.0;
  emptyCmd.y = 0.0;
  emptyCmd.z = 0.0;
  
  // Initialize acceleration command
  accelerationCmd.x = 0.0;
  accelerationCmd.y = 0.0;
  accelerationCmd.z = 0.0;

  // Initialize first trajectory point as the measured position
  trajectory_msgs::JointTrajectoryPoint firstTrajectoryPoint;

  measuredStates = getLastMeasuredStates();
  eulerAngles = getLastEulerAngles();
  position = measuredStates.getVectPos();

  // Initialize yawPrev in KinematicTransform
  kt.yawTargetPrev = eulerAngles.z;

	jointTrajectory.header.stamp = ros::Time::now();
	
  firstTrajectoryPoint.positions.push_back(position.x);
  firstTrajectoryPoint.positions.push_back(position.y);
  firstTrajectoryPoint.positions.push_back(position.z - .01f);
  firstTrajectoryPoint.positions.push_back(eulerAngles.z);
  firstTrajectoryPoint.velocities.push_back(0.);
  firstTrajectoryPoint.velocities.push_back(0.);
  firstTrajectoryPoint.velocities.push_back(0.);
  firstTrajectoryPoint.accelerations.push_back(0.);
  firstTrajectoryPoint.accelerations.push_back(0.);
  firstTrajectoryPoint.accelerations.push_back(0.);
  firstTrajectoryPoint.time_from_start = ros::Duration(.0);

  jointTrajectory.points.push_back(firstTrajectoryPoint);
	
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

  ROS_INFO("*** Real test: enable the offboard control and arm the drone with the remote controller ***");
  ROS_INFO("*** Estimators are reset as long as offboard control is disabled & no trajectory is being broadcasted ***");
  ROS_INFO("*** Have a safe flight. ***");

  while (ros::ok())
  {

    /****************************Manage Drone Mode*****************************/
    // Only in simulated environment
    //In real world, the mode are managed by the pilot with the controller
    if (simulated_env)
    {
      if (droneState.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
      {
        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
          ROS_INFO("OFFBOARD ENABLED");
        last_request = ros::Time::now();
      }
      else
      {
        if (!droneState.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
          if (arming_client.call(arm_cmd) && arm_cmd.response.success)
            ROS_INFO("VEHICLE ARMED");
          last_request = ros::Time::now();
        }
      }
    }
    /**************************************************************************/

    /*******************************Update Loop********************************/
    // Compute dt and save time
    dt = (ros::Time::now().toNSec() - time.toNSec()) / 1000000000.0f;
    time = ros::Time::now();

    // Set reset flag to true as long as drone is not: armed, offboard and first trajectory point is received
    if (!(droneState.armed && (droneState.mode == "OFFBOARD") && isjointTrajectoryFirstCallback)) reset = true;

    // Get the last measured state
    eulerAngles = getLastEulerAngles();
    measuredStates = getLastMeasuredStates();

    // If the measure didn't change cause EKF rate is slower than control rate, use a filtered predicted states instead
    if(measuredStates.x.position == previousMeasuredStates.x.position)
    {
      filterPercent += 0.75f;
      filterPercent = std::max(filterPercent, 1.0f);
      measuredStates.filterStates(predictedStates,filterPercent);
    }
    else
    {
      filterPercent = 0.75f;
      measuredStates.filterStates(predictedStates,filterPercent); // For smooth behavior
      filterPercent = 0.0f;
      previousMeasuredStates.fillStates(measuredStates);
    }

    // Get the estimated state from measurements and previous estimation & command
    predictedStates.fillStates(se.process(dt, measuredStates.getVectPos(), predictedStates, accelerationCmd));

    // Get the acceleration state from another derivator estimator. Cmd = 0 here for simplicity.
    predictedStatesAcc.fillStates(seAcc.process(dt, predictedStates.getVectSpeed(), predictedStatesAcc, emptyCmd));
    predictedStates.replaceAcc(predictedStatesAcc.getVectSpeed());

    if (!useStatesObserver)
    {
      estimatedStates = measuredStates;
      estimatedStates.x.uncertainties = predictedStates.x.uncertainties;
      estimatedStates.y.uncertainties = predictedStates.y.uncertainties;
      estimatedStates.z.uncertainties = predictedStates.z.uncertainties;
    }
    else
    {
      estimatedStates = predictedStates;
    }

    // Get the next trajectory point and update the target state
    trajectoryPoint = getNextTrajectoryPoint(time.toSec());
    targetStates = getState(trajectoryPoint);

    // Get the yaw from the trajectoryPoint
    yaw = trajectoryPoint.positions[3];

    // Replace heading in estimatedStates by the measured one (used by display.py)
    estimatedStates.replaceHeading(eulerAngles.z);
    /**************************************************************************/

    /*************************Full State Feedback & KT*************************/
    // Reset estimator and set reset flag to false
    if (reset)
    {
      se.resetEstimations();
      seAcc.resetEstimations();
      fsf.resetIntegrators();
      reset = false;
    }

    // Compute full state feedback control
    accelerationCmd = fsf.process(dt, estimatedStates, targetStates);

    eulerAngles.z = yaw;
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
    /**************************************************************************/

    /***************************Publish Extra Topics***************************/
    estimatedStatesTrajectoryPoint = getJointTrajectoryPoint(estimatedStates);
    targetStatesTrajectoryPoint = getJointTrajectoryPoint(targetStates);
    ekfStatesTrajectoryPoint = getJointTrajectoryPoint(measuredStates);

    estimatedStatesTrajectoryPoint.accelerations[0] = estimatedStates.x.acceleration;
    estimatedStatesTrajectoryPoint.accelerations[1] = estimatedStates.y.acceleration;
    estimatedStatesTrajectoryPoint.accelerations[2] = estimatedStates.z.acceleration;

    estimatedStates_pub.publish(estimatedStatesTrajectoryPoint);
    referenceStates_pub.publish(targetStatesTrajectoryPoint);
    ekfStates_pub.publish(ekfStatesTrajectoryPoint);
    accelerationCmd_pub.publish(accelerationCmd);
    uncertainties_pub.publish(estimatedStates.getVectUncertainties());
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
