#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <trajectory-control/DroneStates.hpp>
#include <trajectory-control/fsf.hpp>
#include <trajectory-control/kinematicTransform.hpp>
#include <trajectory-control/statesEstimator.hpp>

void jointTrajectoryAcquireCallback(const trajectory_msgs::JointTrajectory & msg) {
  ROS_INFO_STREAM("trajectory_control_node: acquire callback jointTrajectory = " << msg);
}

void measuredStatesAcquireCallback(const geometry_msgs::PoseStamped & msg) {
  ROS_INFO_STREAM("trajectory_control_node: acquire callback measuredStates = " << msg);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "trajectory_control_node");
  ros::NodeHandle nh, nh_private("~");

  // Define subscribers
  ros::Subscriber jointTrajectory_sub = nh.subscribe("/mavros/JointTrajectory", 1, &jointTrajectoryAcquireCallback);
  ros::Subscriber measuredStates_sub = nh.subscribe("/mavros/vision_pose/pose", 1, &measuredStatesAcquireCallback);

  // Define publishers
  ros::Publisher attitudeCommand_pub = nh.advertise<geometry_msgs::Vector3>("attitudeCommand", 1);

  // Declare classes & variables
  FullStatesFeedback fsf;
  KinematicTransform kt;
  StatesEstimator se;

  DroneStates measuredStates, predictedStates, targetStates;
  geometry_msgs::Vector3 accelerationCommand, attitudeCommand, eulerAngles;
  float dt;

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
  kt.param.hoverCompensation = nh_private.param<double>("hoverCompensation",0.5f);
  kt.param.mass = nh_private.param<double>("mass",0.287f);
  kt.param.maxAngle = nh_private.param<double>("maxAngle",45.0f);
  kt.param.maxVerticalAcceleration = nh_private.param<double>("maxVerticalAcceleration",4.0f);

  // Rate of the controller
  ros::Rate r = nh_private.param<int>("rate", 10);
  /** END PARAMETERS **/

  ros::Time time = ros::Time::now()- r.expectedCycleTime(); // Initialize previous time in the past in order for dt calculation to be ok

  while (nh.ok())
  {
    //Get actual dt for the control, not expected one.
    //Here we cannot use r.cycleTime cause it doesn't take into account the r.sleep() time, hence it only gives calculation time
  	dt = (ros::Time::now().toNSec() - time.toNSec())/1000000000.0f;
  	time = ros::Time::now(); 

  	/*Debug
  	ROS_INFO_STREAM("******** trajectory_control_node: dt = " << dt << " ********");
  	ROS_INFO_STREAM("******** trajectory_control_node: expected_dt = " << (r.expectedCycleTime().toNSec())/1000000000.0f << " ********");
  	/**/

    measuredStates = predictedStates; // Should be measured state, so either position from MOCAP or from px4's EKF.
    eulerAngles = eulerAngles; // Should be measured angles (roll, pitch, yaw) from attitude estimation

  	//Get the estimated state from measures and previous estimation & command
    if(!useStatesObserver)
    {
    	predictedStates = se.process(dt, measuredStates.getVectPos(), measuredStates, accelerationCommand);
      measuredStates.x.uncertainties = predictedStates.x.uncertainties;
   		measuredStates.y.uncertainties = predictedStates.y.uncertainties;
   		measuredStates.z.uncertainties = predictedStates.z.uncertainties;
   		predictedStates = measuredStates;
    }
    else predictedStates = se.process(dt, measuredStates.getVectPos(), predictedStates, accelerationCommand);

    //Compute full state feedback control
    accelerationCommand = fsf.process(dt, predictedStates, targetStates);

    //Generate (roll, pitch, thrust) command
    //For compatibility with different aircrafts or even terrestrial robots, this should be in its own node
    attitudeCommand = kt.process(accelerationCommand, eulerAngles);

    attitudeCommand_pub.publish(attitudeCommand);

    ros::spinOnce();
    r.sleep();
  }

	return 0;
}
