#ifndef __FSF_HPP__
#define __FSF_HPP__

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include "DroneStates.hpp"


/// <summary>
/// Respresent the current drone states used for Full State Feedback.
/// </summary>
class FSFParameters
{
    public:
	float Ki;// = 0.0f;
	float Kp;// = 0.3f;
	float Ks;// = 2.5f;
	float KpStep;// = 1.4f;
	float KsStep;// = 1.74f;
	float Ku;// = 1.0f;
  float Ka;

	bool useIntegrator;// = false;
	bool useFeedForward;// = true;

	FSFParameters();
	FSFParameters(float kp, float ks, float ku);
	FSFParameters(float kp, float ks, float ku, float ki);
  FSFParameters(float kp, float ks, float ku, float ki, float kpStep, float ksStep);
  FSFParameters(float kp, float ks, float ku, float ki, float kpStep, float ksStep, float ka);
	~FSFParameters();
};

class FSF1D
{
    public:
	FSFParameters param;

	bool stepMode1D;// = false;

	float iError;// = 0f;
	float pError;//  = 0f;
	float sError;//  = 0f;
  float uError;//  = 0f;
  float aError;//  = 0f;

	void resetIntegrator();

	/// <summary>
	/// Apply a full state feedback with an optional acceleration feedforward and integral action
	/// </summary>
	/// <param name="dt">Dt.</param>
	/// <param name="current">current drone states.</param>
	/// <param name="target">target drone states.</param>
  float process (float dt, DS1D current, DS1D target);
  float processAcc (float dt, DS1D current, DS1D target);

    	void updateParam(FSFParameters fsfParam);

    	FSF1D();
    	FSF1D(FSFParameters parameters);
    	~FSF1D();
};

class FullStatesFeedback
{
    public:
	bool stepMode;

	FSF1D x,y,z;
	void resetIntegrators();
  geometry_msgs::Vector3 process (float dt, DroneStates current, DroneStates target);
  geometry_msgs::Vector3 processAcc (float dt, DroneStates current, DroneStates target);

    	FullStatesFeedback();
    	~FullStatesFeedback();
};

#endif // __FSF_HPP__
