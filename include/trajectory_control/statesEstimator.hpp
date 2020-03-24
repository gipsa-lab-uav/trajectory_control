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
	float Lunc;// = 9.62f;

	//Filter coefficient for actuators
	float filterCoeff;

	//Tuning
	float maxUncertainties;// = 6f;

	SEParameters();
	SEParameters(float lpos, float lspeed, float lunc, float filtercoeff);
	SEParameters(float lpos, float lspeed, float lunc, float filtercoeff, float maxUnc);
	~SEParameters();
};

class SE1D
{
    public:
	SEParameters param;// = new SEParameters();
	bool reset;// = false;

	/// <summary>
	/// Estimates the drone states with a Luenenberg extended derivator observer.
  /// State is position, speed & uncertainties (both model & disturbances)
  /// but it can be used to derivate any variable if cmd = 0.
	/// </summary>
	/// <param name="dt">Dt.</param>
	/// <param name="current">current drone states.</param>
	/// <param name="target">target drone states.</param>
	DS1D process (float dt, float measuredPos, DS1D predicted, float cmd);
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
	void resetEstimations();
  geometry_msgs::Vector3 firstOrderFilterCmd(geometry_msgs::Vector3 cmdPrev,geometry_msgs::Vector3 cmdCurrent);
	StatesEstimator();
	~StatesEstimator();
};

#endif // __STATES_ESTIMATOR_HPP__
