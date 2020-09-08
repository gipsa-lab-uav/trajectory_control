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
    float maxThrottle; //=1.0f
    bool compensateYaw;
    float Ky;
    
    KTParameters();
    KTParameters(float Komp, float m,
		 float angleMax, float maxVerticalAcc,
		 float Kyaw, float maxT);
    ~KTParameters();
};

class KinematicTransform
{
public:
    KTParameters param;
    float yawTargetPrev;
    
    geometry_msgs::Vector3 process(float dt, geometry_msgs::Vector3 accelerations, geometry_msgs::Vector3 radAngles);
    static float clamp(float value, float lowBound, float upperBound);
    
    KinematicTransform();
    ~KinematicTransform();
};

#endif // ____KINEMATIC_TRANSFORM_HPP__
