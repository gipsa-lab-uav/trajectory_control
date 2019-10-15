#include "trajectory-control/kinematicTransform.hpp"

KTParameters::KTParameters()
{
    Kompensation = 0.5f;
    mass = 0.287f;
    maxAngle = 45.0f;
    maxVerticalAcceleration = 4.0f,
    G = 9.81f;
}

KTParameters::KTParameters(float Komp, float m, float angleMax, float maxVerticalAcc)
{
    Kompensation = Komp;
    mass = m;
    maxAngle = angleMax;
    maxVerticalAcceleration = maxVerticalAcc;
}
KTParameters::~KTParameters(){}

KinematicTransform::KinematicTransform()
{


}
KinematicTransform::~KinematicTransform(){}

geometry_msgs::Vector3 KinematicTransform::process(geometry_msgs::Vector3 accelerations, geometry_msgs::Vector3 droneEulerAngles)
{
    // outputCmd(x,y,z) <=> (roll, pitch, thrust)
    geometry_msgs::Vector3 outputCmd;

    float Rad2Deg = 180.0f / 3.14f;

    float cosRoll = cos(droneEulerAngles.x);
    float cosPitch = cos(droneEulerAngles.y);
    float cosYaw = cos(droneEulerAngles.z);
    float sinYaw = sin(droneEulerAngles.z);

    float thrust = param.mass * (accelerations.z + param.G) / (cosRoll * cosPitch);

    //if we are going in freefall, we will do it horizontally, it will be easier to apply thrust to stabilize the drone when wanted
    if (thrust <= 0.2f * param.mass)
    {
        outputCmd.x = 0.0f;
        outputCmd.y = 0.0f;
    }
    else
    {
        // In the regular frame we have:
        // roll = std::Asin(mass / thrust * ( cmdDrone.translation.x * sinYaw - cmdDrone.translation.y * cosYaw ) );
        // pitch = std::Asin(mass / thrust * ( cmdDrone.translation.x * cosYaw + cmdDrone.translation.y * sinYaw) ) );
        // Yaw shouldn't change, so we keep the cos&sin as is, but we have x_reg = z_unity ; y_reg = -x_unity ; z_reg = y_unity
        outputCmd.x = asin(KinematicTransform::clamp(param.mass * (accelerations.x * sinYaw - accelerations.y * cosYaw) / thrust, -1.0f,1.0f));
        outputCmd.y = asin(KinematicTransform::clamp(param.mass * (accelerations.x * cosYaw + accelerations.y * sinYaw) / thrust, -1.0f,1.0f));
    }

    // Clamp to max values
    outputCmd.x = KinematicTransform::clamp((outputCmd.x * Rad2Deg), -param.maxAngle, param.maxAngle);
    outputCmd.y = KinematicTransform::clamp((outputCmd.y * Rad2Deg), -param.maxAngle, param.maxAngle);
    outputCmd.z = KinematicTransform::clamp(thrust, 0.0f, (param.G + param.maxVerticalAcceleration) * param.mass);

    // Map outputs between -1 and 1.
    outputCmd.x = outputCmd.x / param.maxAngle;
    outputCmd.y = outputCmd.y / param.maxAngle;
    outputCmd.z = outputCmd.z * (1.0f + param.Kompensation) / (param.mass * param.G) -1.0f;

    return outputCmd;
}

float KinematicTransform::clamp(float value, float lowBound, float upperBound)
{
    if (value < lowBound){
        return lowBound;
    }
    else if (value > upperBound){
        return upperBound;
    }
    else{
        return value;
    }
}
