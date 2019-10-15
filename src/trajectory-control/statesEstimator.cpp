#include "trajectory-control/statesEstimator.hpp"


SEParameters::SEParameters()
{
	Lpos = 1.03f;
	Lspeed = 10.7f;
	Lunc = 9.62f;
	filterCoeff = 0.95f;

	overSamplingFactor = 1;
	maxUncertainties= 10.0f;

	manualReset = false;
}
SEParameters::SEParameters(float lpos, float lspeed, float lunc, float filtercoeff)
{
	Lpos = lpos;
	Lspeed = lspeed;
	Lunc = lunc;
	filterCoeff = filtercoeff;

	overSamplingFactor = 1;
	maxUncertainties= 10.0f;

	manualReset = false;
}
SEParameters::SEParameters(float lpos, float lspeed, float lunc, float filtercoeff, int oversample, float maxUnc)
{
	Lpos = lpos;
	Lspeed = lspeed;
	Lunc = lunc;
	filterCoeff = filtercoeff;

	overSamplingFactor = oversample;
	maxUncertainties = maxUnc;

	manualReset = false;
}
SEParameters::~SEParameters(){}

SE1D::SE1D()
{
    	reset = false; //not data
}
SE1D::~SE1D(){}

DS1D SE1D::process (float dt, float measuredPos, DS1D predicted, float cmd)
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
	newStates.position = Apos_pos*predicted.position + Apos_speed *predicted.speed; // + Apos_unc*predicted.uncertainties;
	newStates.speed = Aspeed_speed*predicted.speed + Aspeed_unc *predicted.uncertainties; // + Aspeed_pos*predicted.position;
	newStates.uncertainties = Aunc_unc *predicted.uncertainties; // + Aunc_pos*predicted.position + Aunc_speed*predicted.speed;

	//+B*U
	newStates.speed += Bacc *cmd;


	//+L(y - C*Xest)
	newStates.position += param.Lpos * (measuredPos - Cpos * predicted.position);
	newStates.speed += param.Lspeed * (measuredPos - Cpos * predicted.position);
	newStates.uncertainties += param.Lunc * (measuredPos - Cpos * predicted.position);

	//Clamp the uncertainties
	newStates.uncertainties = std::min(newStates.uncertainties, param.maxUncertainties);

	if (reset == true)
	{
	    newStates.position = measuredPos;
	    newStates.speed = 0.0f;
	    newStates.uncertainties = 0.0f;
	    reset = false;
	}
	if (param.manualReset == true)
	{
	    newStates.position = measuredPos;
	    newStates.speed = 0.0f;
	    newStates.uncertainties = 0.0f;
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
	z.param.filterCoeff = 0.15f;
	cmdApplied.x = 0.0f;
	cmdApplied.y = 0.0f;
	cmdApplied.z = 0.0f;
	cmdAppliedPrev.x = 0.0f;
	cmdAppliedPrev.y = 0.0f;
	cmdAppliedPrev.z = 0.0f;

}
StatesEstimator::~StatesEstimator(){}

DroneStates StatesEstimator::process(float dt, geometry_msgs::Vector3 dronePosition, DroneStates predicted, geometry_msgs::Vector3 cmd)
{

	// Filter command to fit actuator response: first order for vertical response, second order for horizontal
	cmdApplied = firstOrderFilterCmd(cmdApplied, cmdAppliedPrev); //update 2nd order
	cmdAppliedPrev = firstOrderFilterCmd(cmdAppliedPrev, cmd); //update 1rst order
	cmdApplied.z = cmdAppliedPrev.z;



	//updateParam(controllerParam);
	float newDt = dt / x.param.overSamplingFactor;
	DroneStates r = predicted;
	DroneStates predictedTemp = predicted;

	for (int i = 0; i < x.param.overSamplingFactor; i++)
	{
		r.x = x.process(newDt, dronePosition.x, predictedTemp.x, cmd.x);
		r.y = y.process(newDt, dronePosition.y, predictedTemp.y, cmd.y);
		r.z = z.process(newDt, dronePosition.z, predictedTemp.z, cmd.z);
		predictedTemp = r;
	}

	return r;
}

void StatesEstimator::resetEstimations()
{
	x.resetEstimation();
	y.resetEstimation();
	z.resetEstimation();
}

geometry_msgs::Vector3 StatesEstimator::firstOrderFilterCmd(geometry_msgs::Vector3 cmdT_1,geometry_msgs::Vector3 cmdCurrent)
{
	geometry_msgs::Vector3 cmdOut;

        cmdOut.x = x.param.filterCoeff * cmdT_1.x + (1.0f - x.param.filterCoeff) * cmdCurrent.x;
        cmdOut.y = y.param.filterCoeff * cmdT_1.y + (1.0f - y.param.filterCoeff) * cmdCurrent.y;
       	cmdOut.z = z.param.filterCoeff * cmdT_1.z + (1.0f - z.param.filterCoeff) * cmdCurrent.z;

	return cmdOut;

}
