#include "trajectory_control/kinematicTransform.hpp"

KTParameters::KTParameters()
{
    hoverCompensation = 0.3f;
    mass = 1.0f;
    maxAngle = 45.0f;
    maxVerticalAcceleration = 4.0f;
    compensateYaw = true;
    Ky = 0.4f;
}

KTParameters::KTParameters(float Komp, float m, float angleMax, float maxVerticalAcc, float Kyaw)
{
    hoverCompensation = Komp;
    mass = m;
    maxAngle = angleMax;
    maxVerticalAcceleration = maxVerticalAcc;
    Ky = Kyaw;
}

KTParameters::~KTParameters() {}

KinematicTransform::KinematicTransform() {
  yawTargetPrev = 0.0f;
}
KinematicTransform::~KinematicTransform() {}

geometry_msgs::Vector3 KinematicTransform::process(float dt, geometry_msgs::Vector3 accelerations, geometry_msgs::Vector3 droneEulerAngles)
{
    // outputCmd(x,y,z) <=> (roll, pitch, thrust)
    geometry_msgs::Vector3 outputCmd;
    float G = 9.81f;
    float Rad2Deg = 180.0f / 3.14f;
    float yawCompensation;

    float thrustMax = param.mass * (param.maxVerticalAcceleration + G);

    float cosYaw = cos(droneEulerAngles.z);
    float sinYaw = sin(droneEulerAngles.z);

    // Compute yawRate on target values to avoid oscillations
    float yawRate = (droneEulerAngles.z - yawTargetPrev) / dt;
    float maxAngleRad = param.maxAngle / Rad2Deg;

    // Compute & sature thrust force -> update accelerations if needed while keeping unchanged z-axis acceleration for safety
    float thrust = param.mass * sqrt(pow(accelerations.x,2) + pow(accelerations.y,2) + pow(accelerations.z + G,2));

    if (thrust > thrustMax)
    {
        if (accelerations.x != 0.0 | accelerations.y != 0.0)
        {
            float alpha = sqrt((pow(thrustMax,2) / pow(param.mass,2) - pow(accelerations.z + G,2)) / (pow(accelerations.x,2) + pow(accelerations.y,2)));

            accelerations.x = alpha*accelerations.x;
            accelerations.y = alpha*accelerations.y;
        }

        thrust = thrustMax;
    }

    //if we are going in freefall, we will do it horizontally, it will be easier to apply thrust to stabilize the drone when wanted
    if (thrust <= 0.2f * param.mass)
    {
        outputCmd.x = 0.0f;
        outputCmd.y = 0.0f;
    }
    else
    {
        // Compute the angles from the acceleration command (from UAV equations)
        // The clamp is to ensure the quantity inside the asin to be between [-1, 1]
        outputCmd.x = asin(KinematicTransform::clamp(param.mass * (accelerations.x * sinYaw - accelerations.y * cosYaw) / thrust, -1.0f, 1.0f));
        //outputCmd.y = asin(KinematicTransform::clamp(param.mass * (accelerations.x * cosYaw + accelerations.y * sinYaw) / thrust, -1.0f, 1.0f));
        outputCmd.y = atan(param.mass * (accelerations.x * cosYaw + accelerations.y * sinYaw) / (accelerations.z + G));
    }

    // If we're trying to compensate Yaw impact on roll
    // Should probably use UAV forward speed as well
    if(param.compensateYaw)
    {
      yawCompensation = param.Ky * yawRate;
      //std::cout << "yawCompensation: " << yawCompensation << std::endl;
      outputCmd.x = outputCmd.x*(1 - yawCompensation);
      // outputCmd.y = outputCmd.y*(1+yawCompensation);
    }

    // Clamp roll and pitch to maximum angle
    outputCmd.x = KinematicTransform::clamp(outputCmd.x, -maxAngleRad, maxAngleRad);
    outputCmd.y = KinematicTransform::clamp(outputCmd.y, -maxAngleRad, maxAngleRad);

    // Map Throttle value between 0 & 1
    outputCmd.z = KinematicTransform::clamp(thrust * param.hoverCompensation / (param.mass * G), 0.0, 1.0);

    // Update
    yawTargetPrev = droneEulerAngles.z;
    return outputCmd;
}

float KinematicTransform::clamp(float value, float lowBound, float upperBound)
{
    if (value < lowBound)
    {
        return lowBound;
    }
    else if (value > upperBound)
    {
        return upperBound;
    }
    else
    {
        return value;
    }
}
