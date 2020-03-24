#include "ros/ros.h"
#include "std_msgs/String.h"

#include "trajectory_control/fsf.hpp"

#include <sstream>

FSFParameters::FSFParameters()
{
	Ki = 0.0f;
	Kp = 1.72f;
	Ks = 1.86f;
	KpStep = 1.4f;
	KsStep = 1.74f;
	Ku = 1.0f;
	Ka = 1.0f;

	useIntegrator = false;
	useFeedForward = false;
}

FSFParameters::FSFParameters(float kp, float ks, float ku)
{
	Ki = 0.0f;
	Kp = kp;
	Ks = ks;
	KpStep = kp;
	KsStep = ks;
	Ku = ku;
	Ka = 1.0f;

	useIntegrator = false;
	useFeedForward = false;
}

FSFParameters::FSFParameters(float kp, float ks, float ku, float ki)
{
	Ki = ki;
	Kp = kp;
	Ks = ks;
	KpStep = kp;
	KsStep = ks;
	Ku = ku;
	Ka = 1.0f;

	useIntegrator = false;
	useFeedForward = false;
}
FSFParameters::FSFParameters(float kp, float ks, float ku, float ki, float kpStep, float ksStep)
{
	Ki = ki;
	Kp = kp;
	Ks = ks;
	KpStep = kpStep;
	KsStep = ksStep;
	Ku = ku;
	Ka = 1.0f;

	useIntegrator = false;
	useFeedForward = false;
}

FSFParameters::FSFParameters(float kp, float ks, float ku, float ki, float kpStep, float ksStep, float ka)
{
	Ki = ki;
	Kp = kp;
	Ks = ks;
	KpStep = kpStep;
	KsStep = ksStep;
	Ku = ku;
	Ka = ka;

	useIntegrator = false;
	useFeedForward = false;
}

FSFParameters::~FSFParameters() {}

/////////////////////////////////:

FSF1D::FSF1D()
{
	stepMode1D = false;
	iError = 0.0f;
	pError = 0.0f;
	sError = 0.0f;
	uError = 0.0f;
	aError = 0.0f;
}

FSF1D::FSF1D(FSFParameters parameters)
{
	param = parameters;
	stepMode1D = false;
	iError = 0.0f;
	pError = 0.0f;
	sError = 0.0f;
	uError = 0.0f;
	aError = 0.0f;
}

void FSF1D::updateParam(FSFParameters fsfParam)
{
	param = fsfParam;
}

void FSF1D::resetIntegrator()
{
	iError = 0.0f;
}

float FSF1D::process(float dt, DS1D current, DS1D target)
{
	if (dt == 0.0f) // If we have time issues, just keep the current command
		return current.acceleration;

	float cmd;

	// Compute standard errors
	pError = target.position - current.position;
	sError = target.speed - current.speed;
	uError = - current.uncertainties;

	// If we're using different control parameters for step movements & trajectory following
	if (stepMode1D)
	{
		cmd = param.KpStep * pError + param.KsStep * sError + param.Ku * uError;
	}
	else
	{
		cmd = param.Kp * pError + param.Ks * sError + param.Ku * uError;
	}

	// Optional saturated integrator
	if (param.useIntegrator)
	{
		iError += pError * dt;

		if (iError > 0.0f)
		{
			iError = std::min(iError, 10.0f);
		}
		else
		{
			iError = std::max(iError, -10.0f);
		}

		cmd += param.Ki * iError;
	}

	// Feed-forward the acceleration or loop on it
	if (param.useFeedForward)
	{
		cmd += target.acceleration;
	}
	else
	{
		aError = target.acceleration - current.acceleration;
		cmd += param.Ka * aError;
	}
	return cmd;
}

FSF1D::~FSF1D() {}

FullStatesFeedback::FullStatesFeedback()
{
	stepMode = false;
}
FullStatesFeedback::~FullStatesFeedback() {}

void FullStatesFeedback::resetIntegrators()
{
	x.FSF1D::resetIntegrator();
	y.FSF1D::resetIntegrator();
	z.FSF1D::resetIntegrator();
}

void FullStatesFeedback::update1DParameters()
{
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
	if (useFeedForward)
	{
		x.param.useFeedForward = true;
		y.param.useFeedForward = true;
		z.param.useFeedForward = true;
	}
	else
	{
		x.param.useFeedForward = false;
		y.param.useFeedForward = false;
		z.param.useFeedForward = false;
	}
	if (useIntegrator)
	{
		x.param.useIntegrator = true;
		y.param.useIntegrator = true;
		z.param.useIntegrator = true;
	}
	else
	{
		x.param.useIntegrator = false;
		y.param.useIntegrator = false;
		z.param.useIntegrator = false;
	}
}

geometry_msgs::Vector3 FullStatesFeedback::process(float dt, DroneStates current, DroneStates target)
{
	geometry_msgs::Vector3 r;
	update1DParameters();

	r.x = x.process(dt, current.x, target.x);
	r.y = y.process(dt, current.y, target.y);
	r.z = z.process(dt, current.z, target.z);
	return r;
}
