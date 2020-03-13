#ifndef __STATES_ESTIMATOR_HPP__
#define __STATES_ESTIMATOR_HPP__

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>

#include "DroneStates.hpp"

class SEParameters
{
    public:
    //Observer Gains
	float Lpos;// = 1.03f;
	float Lspeed;// = 10.7f;
  float Lacc;
	float Lunc;// = 9.62f;

    //Observer with speed measurement Gains
	float LposPos;
	float LposSpeed;
	float LposUnc;
	float LspeedPos;
	float LspeedSpeed;
	float LspeedUnc;

	//Filter coefficient for actuators
	float filterCoeff;

	//Tuning
	int overSamplingFactor;// = 1;
	float maxUncertainties;// = 10f;

	//Options
	bool usePosEst;// = true;
	bool manualReset;// = false;

	SEParameters();
	SEParameters(float lpos, float lspeed, float lunc, float filtercoeff);
	SEParameters(float lpos, float lspeed, float lunc, float filtercoeff, int oversample, float maxUnc);
	SEParameters(float lpospos, float lposspeed, float lposunc, float lspeedpos, float lspeedspeed, float lspeedunc);
	~SEParameters();
};

class SE1D
{
    public:
	SEParameters param;// = new SEParameters();
	bool reset;// = false;

	/// <summary>
	/// Estimates the drone states
	/// </summary>
	/// <param name="dt">Dt.</param>
	/// <param name="current">current drone states.</param>
	/// <param name="target">target drone states.</param>
	DS1D process (float dt, float measuredPos, DS1D predicted, float cmd);
	DS1D process2 (float dt, float measuredPos, float measuredSpeed, DS1D predicted, float cmd);
  DS1D processAcceleration(float dt, float measuredPos, float measuredAcc, DS1D predicted, float cmd);
	void resetEstimation();
	void updateParam(SEParameters seParam);

	SE1D();
	~SE1D();

};

class StatesEstimator
{
    public:
	SE1D x,y,z;
	geometry_msgs::Vector3 cmdApplied, cmdAppliedPrev;
	DroneStates process(float dt, geometry_msgs::Vector3 dronePosition, DroneStates predicted, geometry_msgs::Vector3 cmd);
	DroneStates process2(float dt, geometry_msgs::Vector3 dronePosition, geometry_msgs::Vector3 droneSpeed, DroneStates predicted, geometry_msgs::Vector3 cmd);
  DroneStates processAcceleration(float dt, DroneStates measured, DroneStates predicted, geometry_msgs::Vector3 cmd);
	void resetEstimations();
  geometry_msgs::Vector3 firstOrderFilterCmd(geometry_msgs::Vector3 cmdT_1,geometry_msgs::Vector3 cmdCurrent);
  geometry_msgs::Vector3 computeAccelerations(geometry_msgs::Vector3 droneEulerAngles, float appliedThrust);
	StatesEstimator();
	~StatesEstimator();
};

#endif // __STATES_ESTIMATOR_HPP__
