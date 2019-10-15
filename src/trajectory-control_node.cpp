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

  float dt = .1;

  DroneStates state_current, state_predicted, state_target;
  FullStatesFeedback fsf;
  KinematicTransform kT;
  StatesEstimator estimator;

  geometry_msgs::Vector3 cmd, cmdkT, eulerAng;

  while (n.ok())
  {
    ROS_INFO("******************* This is a test *******************");

    cmd = fsf.process(dt, state_current, state_target);
    cmdkT = kT.process(cmd, eulerAng);
    state_predicted = estimator.process(dt, state_current.getVectPos(), state_predicted, cmd);
    state_current = state_predicted;

    ros::spinOnce();
    r.sleep();
  }

	return 0;
}
