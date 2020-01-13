#include "cable_plugin/cable_visual_plugin.hpp"

namespace gazebo  {
  CableVisualPlugin::CableVisualPlugin() {
  }

  CableVisualPlugin::~CableVisualPlugin() {
  }

  void CableVisualPlugin::Load( rendering::VisualPtr _visual, sdf::ElementPtr _sdf ) {

    gzdbg << "CableVisualPlugin::Load start" << std::endl;

    if (!_visual || !_sdf) {
      gzerr << "No visual or SDF element specified. Plugin won't load." << std::endl;
      return;
    }

    visual_ = _visual;

    if (_sdf->HasElement("robotNamespace"))
      namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
      gzerr << "[gazebo_cable_visual_plugin] Please specify a robotNamespace.\n";

    if (_sdf->HasElement("originCable"))
      origin_cable_ = _sdf->GetElement("originCable")->Get<ignition::math::Vector3d>();
    else
      gzerr << "[gazebo_cable_visual_plugin] Please specify an origin for the cable (originCable).\n";

    if (_sdf->HasElement("linkName"))
      link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
    else
      gzerr << "[gazebo_cable_visual_plugin] Please specify a linkName.\n";

    getSdfParam<double>(_sdf, "linearMassDensity", linear_mass_density_, 0.01);
    getSdfParam<double>(_sdf, "cableTension", cable_tension_, 10);

    if (!ros::isInitialized()){
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
    }

    node_handle_.reset(new ros::NodeHandle(namespace_));

    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<gazebo_msgs::LinkStates>(
        "gazebo/link_states",
        1,
        boost::bind(&CableVisualPlugin::UpdateVisual, this, _1),
        ros::VoidPtr(), &this->queue_);

    sub_ = node_handle_->subscribe(so);
    queue_thread_ = std::thread(std::bind(&CableVisualPlugin::QueueThread, this));

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    updateConnection_ = event::Events::ConnectRender(boost::bind(&CableVisualPlugin::UpdateChild, this));

    gzdbg << "CableVisualPlugin::Load end" << std::endl;
  }

  void CableVisualPlugin::Init() {
    gzdbg << "CableVisualPlugin::Init start" << std::endl;

    line_ = visual_->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
    line_->setMaterial("Gazebo/Purple");
    line_->setVisibilityFlags(GZ_VISIBILITY_GUI);

    force_ = visual_->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
    force_->setMaterial("Gazebo/Yellow");
    force_->setVisibilityFlags(GZ_VISIBILITY_GUI);

    gzdbg << "CableVisualPlugin::Init end" << std::endl;
  }

  void CableVisualPlugin::UpdateChild() {

  }

  void CableVisualPlugin::UpdateVisual(const gazebo_msgs::LinkStatesConstPtr & _msg) {
    if (find_index_nok_) {
      int idx = 0;
      for (auto & name : _msg->name) {
        gzdbg << name << std::endl;
        if (name == link_name_) {
          idx_link_ = idx;
          find_index_nok_ = false;
          break;
        }
      idx += 1;
      }
    }

    line_->Clear();

    line_->AddPoint(origin_cable_);

    line_->AddPoint(ignition::math::Vector3d(
      _msg->pose[idx_link_].position.x,
      _msg->pose[idx_link_].position.y,
      _msg->pose[idx_link_].position.z));

    force_->Clear();

    ignition::math::Vector3d winch_anchor_vector, cable_tension, cable_weight;
    ignition::math::Vector3d anchor_pos(
      _msg->pose[idx_link_].position.x,
      _msg->pose[idx_link_].position.y,
      _msg->pose[idx_link_].position.z);

    force_->AddPoint(anchor_pos);

    winch_anchor_vector = anchor_pos - origin_cable_;
    cable_tension = - cable_tension_ * winch_anchor_vector.Normalized();
    cable_weight = - linear_mass_density_ * winch_anchor_vector.Length() * ignition::math::Vector3d(0, 0, 1);

    force_->AddPoint(anchor_pos + cable_tension + cable_weight);
  }

  void CableVisualPlugin::QueueThread(){
    static const double timeout = .01;
    while (node_handle_->ok()){
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

  GZ_REGISTER_VISUAL_PLUGIN(CableVisualPlugin)
}
