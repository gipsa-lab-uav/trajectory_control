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
      gzerr << "[gazebo_cable_plugin] Please specify a jointName.\n";
    joint_ = model_->GetJoint(joint_name_);
    if (joint_ == NULL)
      gzthrow("[gazebo_cable_plugin] Couldn't find specified joint \"" << joint_name_ << "\".");

    if (_sdf->HasElement("linkName"))
      link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
    else
      gzerr << "[gazebo_cable_plugin] Please specify a linkName.\n";
    link_ = model_->GetLink(link_name_);
    if (link_ == NULL)
      gzthrow("[gazebo_cable_plugin] Couldn't find specified link \"" << link_name_ << "\".");

    if (_sdf->HasElement("originCable"))
      origin_cable_ = _sdf->GetElement("originCable")->Get<ignition::math::Vector3d>();
    else
      gzerr << "[gazebo_cable_visual_plugin] Please specify an origin for the cable (originCable).\n";

    getSdfParam<double>(_sdf, "linearMassDensity", linear_mass_density_, 0.01);
    getSdfParam<double>(_sdf, "cableTension", cable_tension_, 10);

    gzdbg << "linearMassDensity: " << linear_mass_density_ << std::endl;
    gzdbg << "cableTension: " << cable_tension_ << std::endl;

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
    ignition::math::Vector3d winch_anchor_vector, cable_tension, cable_weight;

    winch_anchor_vector = link_->WorldInertialPose().Pos() - origin_cable_;
    cable_tension = - cable_tension_ * winch_anchor_vector.Normalized();
    cable_weight = - linear_mass_density_ * winch_anchor_vector.Length() * ignition::math::Vector3d(0, 0, 1);

    gzdbg << "anchor_pos: " << link_->WorldInertialPose().Pos() << std::endl;
    gzdbg << "origin_cable_: " << origin_cable_ << std::endl;
    gzdbg << "winch_anchor_vector: " << winch_anchor_vector << std::endl;
    gzdbg << "winch_anchor_vector_unit: " << winch_anchor_vector.Normalized() << std::endl;
    gzdbg << "cable_tension_: " << cable_tension_ << std::endl;
    gzdbg << "cable_tension: " << cable_tension << std::endl;
    gzdbg << "length: " << winch_anchor_vector.Length() << std::endl;
    gzdbg << "linear_mass_density_: " << linear_mass_density_ << std::endl;
    gzdbg << "cable_weight: " << cable_weight << std::endl << std::endl;

    // Link::AddForceAtWorldPosition(const math::Vector3 & _force, const math::Vector3 & _pos)
    // Add a force to the body (link) using a global position (in the inertial frame).
    // Parameters:
    // [in]	_force	Force to add.
    // [in]	_pos	  Position in global coord frame to add the force.
    link_->AddForceAtWorldPosition(cable_tension + cable_weight, ignition::math::Vector3d::Zero);
  }


  GZ_REGISTER_MODEL_PLUGIN(CablePlugin)
}
