#include "trajectory_control/statesEstimator.hpp"

SEParameters::SEParameters()
{
	Lpos = 1.03f;
	Lspeed = 10.7f;
	Lunc = 9.62f;
	filterCoeff = 0.95f;

	LposPos = 0.92f;
	LposSpeed = 0.02f;
	LposUnc = 0.11f;
	LspeedPos = 0.006f;
	LspeedSpeed = 0.69f;
	LspeedUnc = 4.2f;

	overSamplingFactor = 1;
	maxUncertainties = 6.0f;

	manualReset = false;
}
SEParameters::SEParameters(float lpos, float lspeed, float lunc, float filtercoeff)
{
	Lpos = lpos;
	Lspeed = lspeed;
	Lunc = lunc;
	filterCoeff = filtercoeff;

	LposPos = 0.92f;
	LposSpeed = 0.0001f;
	LposUnc = 0.0001f;
	LspeedPos = 0.01f;
	LspeedSpeed = 1.0f;
	LspeedUnc = 8.7f;

	overSamplingFactor = 1;
	maxUncertainties = 10.0f;

	manualReset = false;
}
SEParameters::SEParameters(float lpos, float lspeed, float lunc, float filtercoeff, int oversample, float maxUnc)
{
	Lpos = lpos;
	Lspeed = lspeed;
	Lunc = lunc;
	filterCoeff = filtercoeff;

	LposPos = 0.92f;
	LposSpeed = 0.0001f;
	LposUnc = 0.0001f;
	LspeedPos = 0.01f;
	LspeedSpeed = 1.0f;
	LspeedUnc = 8.7f;

	overSamplingFactor = oversample;
	maxUncertainties = maxUnc;

	manualReset = false;
}
SEParameters::SEParameters(float lpospos, float lposspeed, float lposunc, float lspeedpos, float lspeedspeed, float lspeedunc)
{
	Lpos = 1.03f;
	Lspeed = 10.7f;
	Lunc = 9.62f;
	filterCoeff = 0.95f;

	LposPos = lpospos;
	LposSpeed = lposspeed;
	LposUnc = lposunc;
	LspeedPos = lspeedpos;
	LspeedSpeed = lspeedspeed;
	LspeedUnc = lspeedunc;

	overSamplingFactor = 1;
	maxUncertainties = 6.0f;

	manualReset = false;
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
	newStates.position += param.Lpos * (measuredPos - Cpos * predicted.position);
	newStates.speed += param.Lspeed * (measuredPos - Cpos * predicted.position);
	newStates.uncertainties += param.Lunc * (measuredPos - Cpos * predicted.position);

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
	if (param.manualReset == true)
	{
		newStates.position = measuredPos;
		newStates.speed = 0.0f;
		newStates.uncertainties = 0.0f;
	}

	return newStates;
}

DS1D SE1D::process2(float dt, float measuredPos, float measuredSpeed, DS1D predicted, float cmd)
{
	DS1D newStates(0.0f, 0.0f, 0.0f, 0.0f);

	/** Drone second integrator linear model Parameters: better here or elsewhere? **/
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
	float Cspeed = 1.0f;
	//float Cunc = 0f;
	/*End model parameters*/
	float positionError = measuredPos - Cpos * predicted.position;
	float speedErrror = measuredSpeed - Cspeed * predicted.speed;

	//A*Xest
	newStates.position = Apos_pos * predicted.position + Apos_speed * predicted.speed;		 // + Apos_unc*predicted.uncertainties;
	newStates.speed = Aspeed_speed * predicted.speed + Aspeed_unc * predicted.uncertainties; // + Aspeed_pos*predicted.position;

	//+B*U
	newStates.speed += Bacc * cmd;

	//+L(y - C*Xest)
	newStates.position += param.LposPos * positionError;
	newStates.speed += param.LposSpeed * positionError;

	newStates.position += param.LspeedPos * speedErrror;
	newStates.speed += param.LspeedSpeed * speedErrror;
	//Only update Uncertainties if speed & pos are already decent
	if(positionError < 0.05f && speedErrror < 0.05f) {
		newStates.uncertainties = Aunc_unc * predicted.uncertainties;// + Aunc_pos*predicted.position + Aunc_speed*predicted.speed;
		newStates.uncertainties += param.LposUnc * positionError;
		newStates.uncertainties += param.LspeedUnc * speedErrror;
	}
	else newStates.uncertainties = Aunc_unc * predicted.uncertainties;

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
		newStates.speed = measuredSpeed;
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

	// Filter command to fit actuator response: first order for vertical response, second order for horizontal
	cmdApplied = firstOrderFilterCmd(cmdApplied, cmdAppliedPrev); //update 2nd order
	cmdAppliedPrev = firstOrderFilterCmd(cmdAppliedPrev, cmd);	//update 1rst order
	cmdApplied.z = cmdAppliedPrev.z;

	//updateParam(controllerParam);
	float newDt = dt / x.param.overSamplingFactor;
	DroneStates r = predicted;
	DroneStates predictedTemp = predicted;

	for (int i = 0; i < x.param.overSamplingFactor; i++)
	{
		r.x = x.process(newDt, dronePosition.x, predictedTemp.x, cmdApplied.x);
		r.y = y.process(newDt, dronePosition.y, predictedTemp.y, cmdApplied.y);
		r.z = z.process(newDt, dronePosition.z, predictedTemp.z, cmdApplied.z);
		predictedTemp = r;
	}

	return r;
}

DroneStates StatesEstimator::process2(float dt, geometry_msgs::Vector3 dronePosition, geometry_msgs::Vector3 droneSpeed, DroneStates predicted, geometry_msgs::Vector3 cmd)
{

	// Filter command to fit actuator response: first order for vertical response, second order for horizontal
	cmdApplied = firstOrderFilterCmd(cmdApplied, cmdAppliedPrev); //update 2nd order
	cmdAppliedPrev = firstOrderFilterCmd(cmdAppliedPrev, cmd);	//update 1rst order
	cmdApplied.z = cmdAppliedPrev.z;

	//updateParam(controllerParam);
	float newDt = dt / x.param.overSamplingFactor;
	DroneStates r = predicted;
	DroneStates predictedTemp = predicted;

	for (int i = 0; i < x.param.overSamplingFactor; i++)
	{
		r.x = x.process2(newDt, dronePosition.x, droneSpeed.x, predictedTemp.x, cmdApplied.x);
		r.y = y.process2(newDt, dronePosition.y, droneSpeed.y, predictedTemp.y, cmdApplied.y);
		r.z = z.process2(newDt, dronePosition.z, droneSpeed.z, predictedTemp.z, cmdApplied.z);
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

geometry_msgs::Vector3 StatesEstimator::firstOrderFilterCmd(geometry_msgs::Vector3 cmdT_1, geometry_msgs::Vector3 cmdCurrent)
{
	geometry_msgs::Vector3 cmdOut;

	cmdOut.x = x.param.filterCoeff * cmdT_1.x + (1.0f - x.param.filterCoeff) * cmdCurrent.x;
	cmdOut.y = y.param.filterCoeff * cmdT_1.y + (1.0f - y.param.filterCoeff) * cmdCurrent.y;
	cmdOut.z = z.param.filterCoeff * cmdT_1.z + (1.0f - z.param.filterCoeff) * cmdCurrent.z;

	return cmdOut;
}
