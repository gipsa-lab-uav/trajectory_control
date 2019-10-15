#include <ros/ros.h>
#include <ros/console.h>

#include <trajectory-control/fsf.hpp>

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "trajectory_control_node");
  ros::NodeHandle n;

  ros::Rate r = 10;

  while (n.ok())
  {
    ROS_INFO("******************* This is a test *******************");

    ros::spinOnce();
    r.sleep();
  }

	return 0;
}
