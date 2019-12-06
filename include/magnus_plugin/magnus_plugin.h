#include <stdio.h>

#include <boost/bind.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include "gazebo/msgs/msgs.hh"
#include <gazebo/physics/physics.hh>
#include "gazebo/transport/transport.hh"

#include <Eigen/Eigen>
#include <rotors_model/motor_model.hpp>
#include "CommandMotorSpeed.pb.h"
#include "MotorSpeed.pb.h"
#include "Float.pb.h"

#include "common.h"

#define M_PI 3.14159265358979323846

namespace gazebo {
// Default values
static const std::string kDefaultNamespace = "";
static const std::string kDefaultCommandSubTopic = "/gazebo/command/wing_speed";
static const std::string kDefaultWingFailureNumSubTopic = "/gazebo/wing_failure_num";
static const std::string kDefaultWingVelocityPubTopic = "/wing_speed";
static const std::string kDefaultWingSide = "left";

typedef const boost::shared_ptr<const mav_msgs::msgs::CommandMotorSpeed> CommandMotorSpeedPtr;

static constexpr double kDefaulMaxRotVelocity = 1200.0;
static constexpr double kDefaultRotorVelocitySlowdownSim = 10.0;
static constexpr double kDefaultWingMass = 0.06;
static constexpr double kDefaultWingRadius = 0.025;
static constexpr double kDefaultWingLength = 0.15;
static constexpr double kDefaultMomentConstant = 0.016;
static constexpr double kDefaultMotorConstant = 8.54858e-06;

class MagnusPlugin : public MotorModel, public ModelPlugin {
  public:
    MagnusPlugin()
        : ModelPlugin(),
          MotorModel(),
          command_sub_topic_(kDefaultCommandSubTopic), 
          motor_failure_sub_topic_(kDefaultWingFailureNumSubTopic),
          motor_speed_pub_topic_(kDefaultWingVelocityPubTopic),
          wing_side_(kDefaultWingSide),
          wing_mass_(kDefaultWingMass),
          wing_radius_(kDefaultWingRadius),
          wing_length_(kDefaultWingLength),
          moment_constant_(kDefaultMomentConstant),
          motor_constant_(kDefaultMotorConstant),
          wing_number_(0),
          wing_failure_number_(0),
          ref_wing_rot_vel_(0.0),
          rotor_velocity_slowdown_sim_(kDefaultRotorVelocitySlowdownSim),
          max_rot_velocity_(kDefaulMaxRotVelocity) {
    }

    virtual ~MagnusPlugin();
    virtual void InitializeParams();
    virtual void Publish();

  protected:
    virtual void UpdateForcesAndMoments();
    virtual void UpdateMotorFail();
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void OnUpdate(const common::UpdateInfo&_info);

  private:
    std::string command_sub_topic_;
    std::string motor_failure_sub_topic_;
    std::string joint_name_;
    std::string link_name_;
    std::string motor_speed_pub_topic_;
    std::string namespace_;
    std::string wing_side_;

    int wing_failure_number_;
    int wing_number_;
    int tmp_motor_num; // A temporary variable used to print msg

    double max_rot_velocity_;
    double rotor_velocity_slowdown_sim_;
    double ref_wing_rot_vel_;
    double wing_mass_;
    double wing_radius_;
    double wing_length_;
    double moment_constant_;
    double motor_constant_;

    transport::NodePtr node_handle_;
    transport::PublisherPtr motor_velocity_pub_;
    transport::SubscriberPtr command_sub_;
    transport::SubscriberPtr motor_failure_sub_; 

    physics::ModelPtr model_;
    physics::JointPtr joint_;
    physics::LinkPtr link_;
    event::ConnectionPtr updateConnection_;

    std_msgs::msgs::Float turning_velocity_msg_;
    void VelocityCallback(CommandMotorSpeedPtr &rot_velocities);
    void MotorFailureCallback(const boost::shared_ptr<const msgs::Int> &fail_msg);

};
}
