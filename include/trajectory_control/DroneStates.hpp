#ifndef __DRONE_STATES_HPP__
#define __DRONE_STATES_HPP__

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

/// <summary>
/// Respresent the current drone states used for Full State Feedback.
/// </summary>

class DS1D {
    public:
    	float position;
    	float speed;
    	float acceleration;
    	float uncertainties;
    	DS1D();
    	DS1D(float pos, float speedC, float acc, float unc);
	DS1D(const DS1D& otherState);
	~DS1D();
};

class DroneStates {

	/// <summary>
	/// Respresent the current drone states used for 1 dimension of Full State Feedback.
	/// </summary>
	public:
		DS1D x;
		DS1D y;
		DS1D z;

		DroneStates();
		DroneStates(DS1D xNew, DS1D yNew, DS1D zNew);
		~DroneStates();

		void replacePos(geometry_msgs::Vector3 position);
		void replacePosAndSpeed(geometry_msgs::Vector3 position, geometry_msgs::Vector3 speed);
    void replaceSpeed(geometry_msgs::Vector3 speed);
    void replaceAcc(geometry_msgs::Vector3 acceleration);
		geometry_msgs::Vector3 getVectPos();
		geometry_msgs::Vector3 getVectSpeed();
		geometry_msgs::Vector3 getVectAcceleration();
		geometry_msgs::Vector3 getVectUncertainties();
		void fillStates (geometry_msgs::Vector3 pos, geometry_msgs::Vector3 speed, geometry_msgs::Vector3 acc);
		void fillStates (geometry_msgs::Vector3 pos, geometry_msgs::Vector3 speed, geometry_msgs::Vector3 acc, geometry_msgs::Vector3 uncertainties);
};

#endif // __DRONE_STATES_HPP__
