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
      if (!(wing_side_ == "left" || wing_side_ == "right"))
        gzerr << "[gazebo_magnus_wing_model] Please only use 'left' or 'right' as wingSide.\n";
    else
      gzerr << "[gazebo_magnus_wing_model] Please specify a wingSide.\n";

    if (_sdf->HasElement("wingNumber"))
      wing_number_ = _sdf->GetElement("wingNumber")->Get<int>();
    else {
      gzwarn << "[gazebo_magnus_wing_model] No wingNumber found, default used: 0 for 'left' wingSide, 1 for 'right' wingSide.\n";
      if (wing_side_ == "left")
        wing_number_ = 0;
      else if (wing_side_ == "right")
        wing_number_ = 1;
      gzwarn << "[gazebo_magnus_wing_model] wingNumber is (default == " << wing_number_ << ") for the wingSide (" << wing_side_ << ").\n";
    }

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

  void MagnusRoll::OnUpdate(const common::UpdateInfo & _info) {
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

  void MagnusRoll::UpdateForcesAndMoments() {
    // double force;
    double real_wing_velocity;
    // double scalar;
    double vel;
    double spin_ratio;
    double abs_wind_app_vel;
    double drag_coeff_XZ;
    double lift_coeff;
    double drag_coeff_Y;
    double drag_force_XZ;
    double drag_force_Y;
    double wing_inertia;

    double rho_air = 1.2041;
    double surface_area = 2.0*wing_radius_*wing_length_;
    double lat_surface_area = M_PI*std::pow(wing_radius_,2);

    ignition::math::Pose3d pose_difference;
    ignition::math::Vector3d air_drag;
    ignition::math::Vector3d air_lift;
    ignition::math::Vector3d body_velocity;
    ignition::math::Vector3d body_velocity_perpendicular;
    ignition::math::Vector3d drag_torque;
    ignition::math::Vector3d drag_torque_parent_frame;
    ignition::math::Vector3d joint_axis;
    ignition::math::Vector3d rolling_moment;

    ignition::math::Vector3d wind_app_vel_inertial;
    ignition::math::Vector3d wind_app_vel;
    ignition::math::Vector3d vel_dir;
    ignition::math::Vector3d vel_dir_perpendicular;
    ignition::math::Vector3d lift_force;
    ignition::math::Vector3d drag_force;
    ignition::math::Vector3d gyro_torque;

    physics::Link_V parent_links;


    wind_vel = link_->RelativeWindLinearVel();
    omega = link_->RelativeAngularVel();
    vel_inertial = link__->WorldLinearVel();
    wind_vel_inertial = link_->WorldWindLinearVel();


    wind_app_vel_inertial = wind_vel - vel_inertial;
    wind_app_vel = wind_app_vel_inertial * foo();     // transform inertial to body
    abs_wind_app_vel = ignition::math::Vector3d(wind_app_vel.X(), 0, wind_app_vel.Z()).Length();

    if (abs_wind_app_vel > 0)
      spin_ratio = wing_radius_*max_rot_velocity_/abs_wind_app_vel;
    else
      spin_ratio = 0;

    spin_ratio = ignition::math::clamp(spin_ratio, 0.0, 8.0);

    drag_coeff_XZ = -0.0211*std::pow(spin_ratio, 3) + 0.1873*std::pow(spin_ratio, 2) + 0.1183*spin_ratio + 0.5;
    lift_coeff = 0.0126*std::pow(spin_ratio, 4) - 0.2004*std::pow(spin_ratio, 3) + 0.7482*std::pow(spin_ratio, 2) + 1.3447*spin_ratio;
    drag_coeff_Y = 0.8;

    vel_dir = vel_inertial.Normalize();
    vel_dir_perpendicular = vel_dir.Perpendicular().Normalize();  // Should use cross product to find lift direction

    lift_force = 0.5*rho_air*surface_area*lift_coeff*(std::pow(wind_app_vel_inertial.Length(), 2))*vel_dir_perpendicular;
    drag_force_XZ = 0.5*rho_air*surface_area*drag_coeff_XZ*(std::pow(wind_app_vel_inertial.Length(), 2));
    drag_force_Y = 0.5*rho_air*lat_surface_area*drag_coeff_Y*(std::pow(wind_app_vel_inertial.Length(), 2));
    drag_force.Set(drag_force_XZ*vel_dir.X(), drag_force_Y, drag_force_XZ*vel_dir.Z());

    // Omega is angular speed on roll, pitch and yaw as (1,2,3) - respectively
    wing_inertia  = (wing_mass_*std::pow(wing_radius_, 2))/4.0 + (wing_mass_*std::pow(wing_length_, 2))/12.0;
    gyro_torque.Set(wing_inertia*max_rot_velocity_*Omega(3), 0, wing_inertia*max_rot_velocity_*Omega(1));

    wing_rot_vel_ = joint_->GetVelocity(0);
    if (wing_rot_vel_ / (2 * M_PI) > 1 / (2 * sampling_time_)) {
      gzerr << "Aliasing on wing [" << wing_number_ << "] might occur. Consider making smaller simulation time steps or raising the rotor_velocity_slowdown_sim_ param.\n";
    }
    real_wing_velocity = wing_rot_vel_ * rotor_velocity_slowdown_sim_;
    // force = real_wing_velocity * motor_constant_;

    // body_velocity = link_->WorldLinearVel();
    // vel = body_velocity.Length();

    // Apply a force to the link.
    // link_->AddRelativeForce(ignition::math::Vector3d(0, 0, force));

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
