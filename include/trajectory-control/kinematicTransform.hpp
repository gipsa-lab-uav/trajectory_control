#ifndef __KINEMATIC_TRANSFORM_HPP__
#define __KINEMATIC_TRANSFORM_HPP__

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <cmath> 

class KTParameters
{
    public:
    float hoverCompensation;// = 0.33f;
    float mass;// = 0.287f;
    float maxAngle;// = 45.0f
    float maxVerticalAcceleration;// = 4.0f,

    KTParameters();
	KTParameters(float Komp, float m, float angleMax, float maxVerticalAcc);
    ~KTParameters();
};

class KinematicTransform
{
    public:
	KTParameters param;

	geometry_msgs::Vector3 process(geometry_msgs::Vector3 accelerations, geometry_msgs::Vector3 radAngles);
	float clamp(float value, float lowBound, float upperBound);

	KinematicTransform();
	~KinematicTransform();
};

#endif // ____KINEMATIC_TRANSFORM_HPP__