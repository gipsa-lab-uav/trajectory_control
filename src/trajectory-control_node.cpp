#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Vector3.h>

#include <trajectory-control/DroneStates.hpp>
#include <trajectory-control/fsf.hpp>
#include <trajectory-control/kinematicTransform.hpp>
#include <trajectory-control/statesEstimator.hpp>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "trajectory_control_node");
  ros::NodeHandle n;
  ros::Rate r = 10;
  float dt = r.expectedCycleTime().toSec();


  // Declare all publishers & subscribers
  ros::Publisher attitudeCommand_pub = n.advertise<geometry_msgs::Vector3>("attitudeCommand", 10);

  DroneStates measured_state, state_predicted, state_target;
  FullStatesFeedback fsf;
  KinematicTransform kT;
  StatesEstimator estimator;

  bool useEKF = false; //TODO: make this a dynamic parameter.

  geometry_msgs::Vector3 accelerationCommand, attitudeCommand, eulerAng;

  while (n.ok())
  {
    ROS_INFO("******************* This is a test *******************");

    //Get actual dt, not expected one
  	dt = r.cycleTime().toSec();

    measured_state = state_predicted; // Should be measured state, so either position from MOCAP or from px4's EKF.
    eulerAng = eulerAng; // Should be measured angles from attitude estimation 

  	//Get the estimated state from measures and previous estimation & command
    state_predicted = estimator.process(dt, measured_state.getVectPos(), state_predicted, accelerationCommand);
    if(useEKF) state_predicted = measured_state; 

    //Compute full state feedback control
    accelerationCommand = fsf.process(dt, state_predicted, state_target);

    //Generate (roll, pitch, thrust) command
    attitudeCommand = kT.process(accelerationCommand, eulerAng);

	attitudeCommand_pub.publish(attitudeCommand);

    ros::spinOnce();
    r.sleep();
  }

	return 0;
}
