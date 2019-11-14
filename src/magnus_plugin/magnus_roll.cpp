#include "magnus_roll.h"
#include <ignition/math.hh>

namespace gazebo  {
  MagnusRoll::~MagnusRoll() {
    updateConnection_->~Connection();
  }

  void MagnusRoll::InitializeParams() {}

  void MagnusRoll::Publish() {
    turning_velocity_msg_.set_data(joint_->GetVelocity(0));
    motor_velocity_pub_->Publish(turning_velocity_msg_);
  }

  void MagnusRoll::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    model_ = _model;

    namespace_.clear();

    if (_sdf->HasElement("robotNamespace"))
      namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
      gzerr << "[gazebo_magnus_wing_model] Please specify a robotNamespace.\n";
    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init(namespace_);

    if (_sdf->HasElement("jointName"))
      joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();
    else
      gzerr << "[gazebo_magnus_wing_model] Please specify a jointName, where the wing is attached.\n";
    // Get the pointer to the joint.
    joint_ = model_->GetJoint(joint_name_);
    if (joint_ == NULL)
      gzthrow("[gazebo_magnus_wing_model] Couldn't find specified joint \"" << joint_name_ << "\".");

    if (_sdf->HasElement("linkName"))
      link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
    else
      gzerr << "[gazebo_magnus_wing_model] Please specify a linkName of the wing.\n";
    link_ = model_->GetLink(link_name_);
    if (link_ == NULL)
      gzthrow("[gazebo_magnus_wing_model] Couldn't find specified link \"" << link_name_ << "\".");

    if (_sdf->HasElement("wingSide"))
      wing_side_ = _sdf->GetElement("wingSide")->Get<std::string>();
    else
      gzerr << "[gazebo_magnus_wing_model] Please specify a wingSide.\n";

    if (_sdf->HasElement("wingNumber"))
      wing_number_ = _sdf->GetElement("wingNumber")->Get<int>();
    else
      gzerr << "[gazebo_magnus_wing_model] Please specify a wingNumber.\n";

    getSdfParam<std::string>(_sdf, "commandSubTopic", command_sub_topic_, command_sub_topic_);
    getSdfParam<std::string>(_sdf, "motorSpeedPubTopic", motor_speed_pub_topic_, motor_speed_pub_topic_);

    getSdfParam<double>(_sdf, "maxRotVelocity", max_rot_velocity_, max_rot_velocity_);
    getSdfParam<double>(_sdf, "rotorVelocitySlowdownSim", rotor_velocity_slowdown_sim_, 10);
    getSdfParam<double>(_sdf, "wingMass", wing_mass_, wing_mass_);
    getSdfParam<double>(_sdf, "wingRadius", wing_radius_, wing_radius_);
    getSdfParam<double>(_sdf, "wingLength", wing_length_, wing_length_);

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&MagnusRoll::OnUpdate, this, _1));
    // command_sub_ = node_handle_->Subscribe<mav_msgs::msgs::CommandMotorSpeed>("~/" + model_->GetName() + command_sub_topic_, &MagnusRoll::VelocityCallback, this);
    motor_failure_sub_ = node_handle_->Subscribe<msgs::Int>(motor_failure_sub_topic_, &MagnusRoll::MotorFailureCallback, this);
    motor_velocity_pub_ = node_handle_->Advertise<std_msgs::msgs::Float>("~/" + model_->GetName() + motor_speed_pub_topic_, 1);
  }

  void MagnusRoll::OnUpdate(const common::UpdateInfo& _info) {
    sampling_time_ = _info.simTime.Double() - prev_sim_time_;
    prev_sim_time_ = _info.simTime.Double();
    UpdateForcesAndMoments();
    UpdateMotorFail();
    Publish();
  }

  void MagnusRoll::VelocityCallback(CommandMotorSpeedPtr &rot_velocities) {
    if(rot_velocities->motor_speed_size() < wing_number_) {
      std::cout  << "You tried to access index " << wing_number_
        << " of the WingSpeed message array which is of size " << rot_velocities->motor_speed_size() << "." << std::endl;
    } else ref_wing_rot_vel_ = std::min(static_cast<double>(rot_velocities->motor_speed(wing_number_)), static_cast<double>(max_rot_velocity_));
  }

  void MagnusRoll::MotorFailureCallback(const boost::shared_ptr<const msgs::Int> &fail_msg) {
    wing_Failure_Number_ = fail_msg->data();
  }

  void GazeboWingModel::UpdateForcesAndMoments() {
    // double force;
    double real_wing_velocity;
    // double scalar;
    double vel;
    double spin_ratio;
   
    ignition::math::Pose3d pose_difference;
    ignition::math::Vector3d air_drag;
    ignition::math::Vector3d body_velocity;
    ignition::math::Vector3d body_velocity_perpendicular;
    ignition::math::Vector3d drag_torque;
    ignition::math::Vector3d drag_torque_parent_frame;
    ignition::math::Vector3d joint_axis;
    ignition::math::Vector3d rolling_moment;
    physics::Link_V parent_links;

    spin_ratio = wing_mass_*max_rot_velocity_/norm(WappVelMag); // update

    wing_rot_vel_ = joint_->GetVelocity(0);
    if (wing_rot_vel_ / (2 * M_PI) > 1 / (2 * sampling_time_)) {
      gzerr << "Aliasing on wing [" << wing_number_ << "] might occur. Consider making smaller simulation time steps or raising the rotor_velocity_slowdown_sim_ param.\n";
    }
    real_wing_velocity = wing_rot_vel_ * rotor_velocity_slowdown_sim_;
    // force = real_wing_velocity * motor_constant_;

    body_velocity = link_->WorldLinearVel();
    vel = body_velocity.Length();
    // scalar = 1 - vel / 25.0; // at 50 m/s the rotor will not produce any force anymore
    // scalar = ignition::math::clamp(scalar, 0.0, 1.0);

    // Apply a force to the link.
    // link_->AddRelativeForce(ignition::math::Vector3d(0, 0, force));// * scalar));

    // Forces from Philppe Martin's and Erwan Salaün's
    // 2010 IEEE Conference on Robotics and Automation paper
    // The True Role of Accelerometer Feedback in Quadrotor Control
    // - \omega * \lambda_1 * V_A^{\perp}
    joint_axis = joint_->GlobalAxis(0);

    body_velocity_perpendicular = body_velocity - (body_velocity * joint_axis) * joint_axis;
    air_drag = -std::abs(real_wing_velocity) * rotor_drag_coefficient_ * body_velocity_perpendicular;

    // Apply air_drag to link.
    link_->AddForce(air_drag);

    // Moments
    // Getting the parent link, such that the resulting torques can be applied to it.
    parent_links = link_->GetParentJointsLinks();
    // The tansformation from the parent_link to the link_.
    pose_difference = link_->WorldCoGPose() - parent_links.at(0)->WorldCoGPose();

    // drag_torque.Set(0, 0, -turning_direction_ * force * moment_constant_);
    // Transforming the drag torque into the parent frame to handle arbitrary rotor orientations.
    drag_torque_parent_frame = pose_difference.Rot().RotateVector(drag_torque);
    parent_links.at(0)->AddRelativeTorque(drag_torque_parent_frame);
    
    // - \omega * \mu_1 * V_A^{\perp}
    rolling_moment = -std::abs(real_wing_velocity) * rolling_moment_coefficient_ * body_velocity_perpendicular;
    parent_links.at(0)->AddTorque(rolling_moment);

    joint_->SetVelocity(0, turning_direction_ * ref_wing_rot_vel_ / rotor_velocity_slowdown_sim_);
  }


  void MagnusRoll::UpdateMotorFail() {
    if (wing_number_ == wing_Failure_Number_ - 1){
      joint_->SetVelocity(0,0);

      std::cout << "Wing number [" << wing_Failure_Number_ <<"] failed!  [Wing speed = 0]" << std::endl;
      tmp_motor_num = wing_Failure_Number_;
      
    }else if (wing_Failure_Number_ == 0 && motor_number_ ==  tmp_motor_num - 1){
        std::cout << "Wing number [" << tmp_motor_num <<"] running! [Wing speed = (default)]" << std::endl;
    }
  }

  GZ_REGISTER_MODEL_PLUGIN(MagnusRoll)
}
