#include "trajectory_control/DroneStates.hpp"

DS1D::DS1D()
{
	position = 0.0f;
	speed = 0.0f;
	acceleration = 0.0f;
	uncertainties = 0.0f;
}
DS1D::DS1D(float pos, float speedC, float acc, float unc)
{
	position = pos;
	speed = speedC;
	acceleration = acc;
	uncertainties = unc;
}
DS1D::DS1D(const DS1D& otherState)
{
	position = otherState.position;
	speed = otherState.speed;
	acceleration = otherState.acceleration;
	uncertainties = otherState.uncertainties;
}
DS1D::~DS1D(){}

DroneStates::DroneStates()
{
	heading = 0.0f;
}
DroneStates::DroneStates(DS1D xNew, DS1D yNew, DS1D zNew)
{
	x = xNew;
	y = yNew;
	z = zNew;
	heading = 0.0f;
}
DroneStates::~DroneStates(){}

void DroneStates::replacePos(geometry_msgs::Vector3 position)
{
    x.position = position.x;
    y.position = position.y;
    z.position = position.z;
}

void DroneStates::replacePosAndSpeed(geometry_msgs::Vector3 position, geometry_msgs::Vector3 speed)
{
    x.position = position.x;
    y.position = position.y;
    z.position = position.z;

    x.speed = speed.x;
    y.speed = speed.y;
    z.speed = speed.z;
}

void DroneStates::replaceSpeed(geometry_msgs::Vector3 speed)
{
	x.speed = speed.x;
	y.speed = speed.y;
	z.speed = speed.z;
}

void DroneStates::replaceAcc(geometry_msgs::Vector3 acceleration)
{
	x.acceleration = acceleration.x;
	y.acceleration = acceleration.y;
	z.acceleration = acceleration.z;
}

void DroneStates::replaceHeading(float newHeading)
{
	heading = newHeading;
}

geometry_msgs::Vector3 DroneStates::getVectPos()
{
    geometry_msgs::Vector3 r;

    r.x = x.position;
    r.y = y.position;
    r.z = z.position;

    return r;
}

geometry_msgs::Vector3 DroneStates::getVectSpeed()
{
    geometry_msgs::Vector3 r;

    r.x = x.speed;
    r.y = y.speed;
    r.z = z.speed;

    return r;
}

geometry_msgs::Vector3 DroneStates::getVectAcceleration()
{
    geometry_msgs::Vector3 r;

    r.x = x.acceleration;
    r.y = y.acceleration;
    r.z = z.acceleration;

    return r;
}

geometry_msgs::Vector3 DroneStates::getVectUncertainties()
{
    geometry_msgs::Vector3 r;

    r.x = x.uncertainties;
    r.y = y.uncertainties;
    r.z = z.uncertainties;

    return r;
}


void DroneStates::fillStates (geometry_msgs::Vector3 pos, geometry_msgs::Vector3 speed, geometry_msgs::Vector3 acc)
{
    x.position = pos.x;
    x.speed = speed.x;
    x.acceleration = acc.x;
    y.position = pos.y;
    y.speed = speed.y;
    y.acceleration = acc.y;
    z.position = pos.z;
    z.speed = speed.z;
    z.acceleration = acc.z;
}

void DroneStates::fillStates (geometry_msgs::Vector3 pos, geometry_msgs::Vector3 speed, geometry_msgs::Vector3 acc, geometry_msgs::Vector3 uncertainties)
{
    x.position = pos.x;
    x.speed = speed.x;
    x.acceleration = acc.x;
    x.uncertainties = uncertainties.x;
    y.position = pos.y;
    y.speed = speed.y;
    y.acceleration = acc.y;
    y.uncertainties = uncertainties.y;
    z.position = pos.z;
    z.speed = speed.z;
    z.acceleration = acc.z;
    z.uncertainties = uncertainties.z;
}

void DroneStates::fillStates (DroneStates states)
{
    x.position = states.x.position;
    x.speed = states.x.speed;
    x.acceleration = states.x.acceleration;
    x.uncertainties = states.x.uncertainties;
		y.position = states.y.position;
    y.speed = states.y.speed;
    y.acceleration = states.y.acceleration;
    y.uncertainties = states.y.uncertainties;
		z.position = states.z.position;
    z.speed = states.z.speed;
    z.acceleration = states.z.acceleration;
    z.uncertainties = states.z.uncertainties;
}

void DroneStates::filterStates (DroneStates states, float filterPercent)
{
    x.position = (1-filterPercent) * x.position + filterPercent * states.x.position;
    x.speed = (1-filterPercent) * x.speed + filterPercent * states.x.speed;
    x.acceleration = (1-filterPercent) * x.acceleration + filterPercent * states.x.acceleration;
    x.uncertainties = (1-filterPercent) * x.uncertainties + filterPercent * states.x.uncertainties;
		y.position = (1-filterPercent) * y.position + filterPercent * states.y.position;
    y.speed = (1-filterPercent) * y.speed + filterPercent * states.y.speed;
    y.acceleration = (1-filterPercent) * y.acceleration + filterPercent * states.y.acceleration;
    y.uncertainties = (1-filterPercent) * y.uncertainties + filterPercent * states.y.uncertainties;
		z.position = (1-filterPercent) * z.position + filterPercent * states.z.position;
    z.speed = (1-filterPercent) * z.speed + filterPercent * states.z.speed;
    z.acceleration = (1-filterPercent) * z.acceleration + filterPercent * states.z.acceleration;
    z.uncertainties = (1-filterPercent) * z.uncertainties + filterPercent * states.z.uncertainties;
}
