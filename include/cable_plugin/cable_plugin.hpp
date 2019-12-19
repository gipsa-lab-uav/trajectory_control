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

class CablePlugin : public ModelPlugin, public MotorModel {
  public:
    CablePlugin()
        : ModelPlugin(),
          MotorModel(),
          linear_mass_density_(0.01) {
    }

    virtual ~CablePlugin();
    virtual void InitializeParams();
    virtual void Publish();

  protected:
    virtual void UpdateForcesAndMoments();
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void OnUpdate(const common::UpdateInfo&_info);

  private:
    std::string joint_name_;
    std::string link_name_;
    std::string namespace_;

    double linear_mass_density_;

    transport::NodePtr node_handle_;

    physics::ModelPtr model_;
    physics::JointPtr joint_;
    physics::LinkPtr link_;
    event::ConnectionPtr updateConnection_;

};
}
