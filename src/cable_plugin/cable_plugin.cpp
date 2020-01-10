#include "cable_plugin/cable_plugin.hpp"
#include <ignition/math.hh>

namespace gazebo  {
  CablePlugin::~CablePlugin() {
    updateConnection_->~Connection();
  }

  void CablePlugin::InitializeParams() {}

  void CablePlugin::Publish() {
  }

  void CablePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

    gzdbg << "CablePlugin::Load" << std::endl;

    model_ = _model;

    namespace_.clear();

    if (_sdf->HasElement("robotNamespace"))
      namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
      gzerr << "[gazebo_cable_plugin] Please specify a robotNamespace.\n";
    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init(namespace_);

    if (_sdf->HasElement("jointName"))
      joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();
    else
      gzerr << "[gazebo_cable_plugin] Please specify a jointName, where the wing is attached.\n";

    // Get the pointer to the joint.
    joint_ = model_->GetJoint(joint_name_);
    if (joint_ == NULL)
      gzthrow("[gazebo_cable_plugin] Couldn't find specified joint \"" << joint_name_ << "\".");

    if (_sdf->HasElement("linkName"))
      link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
    else
      gzerr << "[gazebo_cable_plugin] Please specify a linkName of the wing.\n";
    link_ = model_->GetLink(link_name_);
    if (link_ == NULL)
      gzthrow("[gazebo_cable_plugin] Couldn't find specified link \"" << link_name_ << "\".");

    getSdfParam<double>(_sdf, "linear_mass_density", linear_mass_density_, 0.01);

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&CablePlugin::OnUpdate, this, _1));
  }

  void CablePlugin::OnUpdate(const common::UpdateInfo & _info) {
    sampling_time_ = _info.simTime.Double() - prev_sim_time_;
    prev_sim_time_ = _info.simTime.Double();
    UpdateForcesAndMoments();
    Publish();
  }

  void CablePlugin::UpdateForcesAndMoments() {
  }


  GZ_REGISTER_MODEL_PLUGIN(CablePlugin)
}
