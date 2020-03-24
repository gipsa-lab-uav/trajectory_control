#include "trajectory_control/statesEstimator.hpp"

SEParameters::SEParameters()
{
	Lpos = 1.03f;
	Lspeed = 10.7f;
	Lunc = 9.62f;
	filterCoeff = 0.95f;

	maxUncertainties = 6.0f;
}
SEParameters::SEParameters(float lpos, float lspeed, float lunc, float filtercoeff)
{
	Lpos = lpos;
	Lspeed = lspeed;
	Lunc = lunc;
	filterCoeff = filtercoeff;

	maxUncertainties = 6.0f;
}
SEParameters::SEParameters(float lpos, float lspeed, float lunc, float filtercoeff, float maxUnc)
{
	Lpos = lpos;
	Lspeed = lspeed;
	Lunc = lunc;
	filterCoeff = filtercoeff;

	maxUncertainties = maxUnc;
}
SEParameters::~SEParameters() {}

SE1D::SE1D()
{
	reset = false; //not data
}
SE1D::~SE1D() {}

DS1D SE1D::process(float dt, float measuredPos, DS1D predicted, float cmd)
{
	DS1D newStates(0.0f, 0.0f, 0.0f, 0.0f);

	/** Drone model Parameters: better here or elsewhere? **/
	// State matrix
	float Apos_pos = 1.0f;
	float Apos_speed = dt;
	//float Apos_unc = 0f;

	//float Aspeed_pos = 0f;
	float Aspeed_speed = 1.0f;
	float Aspeed_unc = dt;

	//float Aunc_pos = 0f;
	//float Aunc_speed = 0f;
	float Aunc_unc = 1.0f;

	// Command vector
	//float Bspeed = 0f;
	float Bacc = dt;
	//float Bunc_dot = 0f;

	// Output/measurement vector
	float Cpos = 1.0f;
	//float Cspeed = 0f;
	//float Cunc = 0f;

	//A*Xest
	newStates.position = Apos_pos * predicted.position + Apos_speed * predicted.speed;		 // + Apos_unc*predicted.uncertainties;
	newStates.speed = Aspeed_speed * predicted.speed + Aspeed_unc * predicted.uncertainties; // + Aspeed_pos*predicted.position;
	newStates.uncertainties = Aunc_unc * predicted.uncertainties;							 // + Aunc_pos*predicted.position + Aunc_speed*predicted.speed;

	//+B*U
	newStates.speed += Bacc * cmd;

	//+L(y - C*Xest)
	float positionMeasurementError = measuredPos - Cpos * predicted.position;
	newStates.position += param.Lpos * positionMeasurementError;
	newStates.speed += param.Lspeed * positionMeasurementError;
	newStates.uncertainties += param.Lunc * positionMeasurementError;

	//Clamp the uncertainties
	if (newStates.uncertainties > 0.0f)
	{
		newStates.uncertainties = std::min(newStates.uncertainties, param.maxUncertainties);
	}
	else
	{
		newStates.uncertainties = std::max(newStates.uncertainties, -param.maxUncertainties);
	}

	if (reset == true)
	{
		newStates.position = measuredPos;
		newStates.speed = 0.0f;
		newStates.uncertainties = 0.0f;
		reset = false;
	}
	return newStates;
}

void SE1D::resetEstimation()
{
	reset = true;
}
void SE1D::updateParam(SEParameters seParam)
{
	param = seParam;
}

StatesEstimator::StatesEstimator()
{
	z.param.filterCoeff = 0.0f;
	cmdApplied.x = 0.0f;
	cmdApplied.y = 0.0f;
	cmdApplied.z = 0.0f;
	cmdAppliedPrev.x = 0.0f;
	cmdAppliedPrev.y = 0.0f;
	cmdAppliedPrev.z = 0.0f;
}
StatesEstimator::~StatesEstimator() {}

DroneStates StatesEstimator::process(float dt, geometry_msgs::Vector3 dronePosition, DroneStates predicted, geometry_msgs::Vector3 cmd)
{
	DroneStates r;
	r.fillStates(predicted);

	// Filter command to fit actuator response: first order for vertical response, second order for horizontal
	cmdApplied = firstOrderFilterCmd(cmdApplied, cmdAppliedPrev); //update 2nd order
	cmdAppliedPrev = firstOrderFilterCmd(cmdAppliedPrev, cmd);	//update 1rst order
	cmdApplied.z = cmdAppliedPrev.z;

	r.x = x.process(dt, dronePosition.x, predicted.x, cmdApplied.x);
	r.y = y.process(dt, dronePosition.y, predicted.y, cmdApplied.y);
	r.z = z.process(dt, dronePosition.z, predicted.z, cmdApplied.z);

	return r;
}

void StatesEstimator::resetEstimations()
{
	x.resetEstimation();
	y.resetEstimation();
	z.resetEstimation();
}

geometry_msgs::Vector3 StatesEstimator::firstOrderFilterCmd(geometry_msgs::Vector3 cmdPrev, geometry_msgs::Vector3 cmdCurrent)
{
	geometry_msgs::Vector3 cmdOut;

	cmdOut.x = x.param.filterCoeff * cmdPrev.x + (1.0f - x.param.filterCoeff) * cmdCurrent.x;
	cmdOut.y = y.param.filterCoeff * cmdPrev.y + (1.0f - y.param.filterCoeff) * cmdCurrent.y;
	cmdOut.z = z.param.filterCoeff * cmdPrev.z + (1.0f - z.param.filterCoeff) * cmdCurrent.z;

	return cmdOut;
}
