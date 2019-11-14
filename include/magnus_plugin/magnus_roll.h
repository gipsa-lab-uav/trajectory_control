#include <stdio.h>

#include <boost/bind.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include "gazebo/msgs/msgs.hh"
#include <gazebo/physics/physics.hh>
#include "gazebo/transport/transport.hh"
#include <rotors_model/motor_model.hpp>

#include "common.h"


namespace gazebo {
// Default values
static const std::string kDefaultNamespace = "";
static const std::string kDefaultCommandSubTopic = "/gazebo/command/wing_speed";
static const std::string kDefaultWingFailureNumSubTopic = "/gazebo/wing_failure_num";
static const std::string kDefaultWingVelocityPubTopic = "/wing_speed";
static const std::string kDefaultWingSide = "left";

typedef const boost::shared_ptr<const mav_msgs::msgs::CommandMotorSpeed> CommandMotorSpeedPtr;

static constexpr double kDefaulMaxRotVelocity = 1200.0;
static constexpr double kDefaultMotorConstant = 8.54858e-06;  // relation between speed squared and force
static constexpr double kDefaultRotorVelocitySlowdownSim = 10.0;

class MagnusRoll : public MotorModel, public ModelPlugin {
  public:
    MagnusRoll()
        : ModelPlugin(),
          MotorModel(),
          command_sub_topic_(kDefaultCommandSubTopic), 
          motor_failure_sub_topic_(kDefaultWingFailureNumSubTopic),
          motor_speed_pub_topic_(kDefaultWingVelocityPubTopic),
          wing_side_(kDefaultWingSide),
          motor_constant_(kDefaultMotorConstant),
          wing_number_(0),
          wing_Failure_Number_(0),
          ref_wing_rot_vel_(0.0),
          rotor_velocity_slowdown_sim_(kDefaultRotorVelocitySlowdownSim),
          max_rot_velocity_(kDefaulMaxRotVelocity) {
    }

    virtual ~MagnusRoll();
    virtual void InitializeParams();
    virtual void Publish();

  private:
    std::string command_sub_topic_;
    std::string motor_failure_sub_topic_;
    std::string joint_name_;
    std::string link_name_;
    std::string motor_speed_pub_topic_;
    std::string namespace_;
    std::string wing_side_;

    int wing_Failure_Number_;
    int wing_number_;
    int tmp_motor_num; // A temporary variable used to print msg

    double max_rot_velocity_;
    double rotor_velocity_slowdown_sim_;
    double ref_wing_rot_vel_;
    double motor_constant_;

};
}
