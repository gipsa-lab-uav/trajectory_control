#include "ros/ros.h"
#include "std_msgs/String.h"

#include "trajectory_control/fsf.hpp"

#include <sstream>

FSFParameters::FSFParameters()
{
	Ki = 0.0f;
  Kp = 0.3f;
	Ks = 2.5f;
	KpStep = 1.4f;
	KsStep = 1.74f;
	Ku = 1.0f;

	useIntegrator = false;
	useFeedForward = true;
}

FSFParameters::FSFParameters(float kp, float ks, float ku)
{
	Ki = 0.0f;
  Kp = kp;
	Ks = ks;
	KpStep = kp;
	KsStep = ks;
	Ku = ku;

	useIntegrator = false;
	useFeedForward = true;

}

FSFParameters::FSFParameters(float kp, float ks, float ku, float ki)
{
	Ki = ki;
  Kp = kp;
	Ks = ks;
	KpStep = kp;
	KsStep = ks;
	Ku = ku;

	useIntegrator = false;
	useFeedForward = true;
}
FSFParameters::FSFParameters(float kp, float ks, float ku, float ki, float kpStep, float ksStep)
{
	Ki = ki;
  Kp = kp;
	Ks = ks;
	KpStep = kpStep;
	KsStep = ksStep;
	Ku = ku;

	useIntegrator = false;
	useFeedForward = true;
}

FSFParameters::~FSFParameters(){}

/////////////////////////////////:

FSF1D::FSF1D()
{
	stepMode1D = false;
	iError = 0.0f;
	pError = 0.0f;
	sError = 0.0f;
	uError = 0.0f;
}

FSF1D::FSF1D(FSFParameters parameters)
{
	param = parameters;
	stepMode1D = false;
	iError = 0.0f;
	pError = 0.0f;
	sError = 0.0f;
	uError = 0.0f;
}

void FSF1D::updateParam(FSFParameters fsfParam)
{
	param = fsfParam;
}

void FSF1D::resetIntegrator()
{
	iError = 0.0f;
}

float FSF1D::process (float dt, DS1D current, DS1D target) {
	if (dt == 0.0f) return current.acceleration;

	float cmd;

	pError = target.position - current.position;
	sError = target.speed - current.speed;
	uError = target.uncertainties - current.uncertainties;

	if (stepMode1D)
	{
		cmd = param.KpStep * pError + param.KsStep * sError + param.Ku * uError;
	}
	else
	{
		cmd = param.Kp * pError + param.Ks * sError + param.Ku * uError;
	}

	if(param.useIntegrator) {
		iError += pError * dt;
		cmd += param.Ki * iError;
	}
	if(param.useFeedForward) {
		cmd += target.acceleration;
	}
	return cmd;
}
FSF1D::~FSF1D(){}



FullStatesFeedback::FullStatesFeedback()
{
	stepMode = false;
}
FullStatesFeedback::~FullStatesFeedback(){}


void FullStatesFeedback::resetIntegrators()
{
	x.FSF1D::resetIntegrator ();
	y.FSF1D::resetIntegrator ();
	z.FSF1D::resetIntegrator ();
}


geometry_msgs::Vector3 FullStatesFeedback::process (float dt, DroneStates current, DroneStates target)
{
	geometry_msgs::Vector3 r;
	if (stepMode)
	{
		x.stepMode1D = true;
		y.stepMode1D = true;
		z.stepMode1D = true;
	}
	else
	{
		x.stepMode1D = false;
		y.stepMode1D = false;
		z.stepMode1D = false;
	}
	r.x = x.process (dt, current.x, target.x);
	r.y = y.process (dt, current.y, target.y);
	r.z = z.process (dt, current.z, target.z);
	return r;
}
