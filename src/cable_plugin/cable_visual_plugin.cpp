#include "cable_plugin/cable_visual_plugin.hpp"
#include <ignition/math.hh>

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
    // link_ = _visual->GetParent();
    // model_ = link_->GetModel();

    if (_sdf->HasElement("robotNamespace"))
      namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
      gzerr << "[gazebo_cable_visual_plugin] Please specify a robotNamespace.\n";

    if (_sdf->HasElement("linkName1"))
      link_name1_ = _sdf->GetElement("linkName1")->Get<std::string>();
    else
      gzerr << "[gazebo_cable_visual_plugin] Please specify a linkName1.\n";
    // link1_ = model_->GetLink(link_name1_);
    // if (link1_ == NULL)
    //   gzthrow("[gazebo_cable_visual_plugin] Couldn't find specified link \"" << link_name1_ << "\".");

    if (_sdf->HasElement("linkName2"))
      link_name2_ = _sdf->GetElement("linkName2")->Get<std::string>();
    else
      gzerr << "[gazebo_cable_visual_plugin] Please specify a linkName2.\n";
    // link2_ = model_->GetLink(link_name2_);
    // if (link2_ == NULL)
    //   gzthrow("[gazebo_cable_visual_plugin] Couldn't find specified link \"" << link_name2_ << "\".");

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
        ros::VoidPtr(),
        &this->queue_);

    sub_ = node_handle_->subscribe(so);
    queue_thread_ = std::thread(std::bind(&CableVisualPlugin::QueueThread, this));

    // node_handle_ = transport::NodePtr(new transport::Node());
    // node_handle_->Init(namespace_);

    // sub_ = node_handle_->Subscribe("link_states", &CableVisualPlugin::UpdateVisual);
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

    gzdbg << "CableVisualPlugin::Init add 1st point" << std::endl;
    line_->AddPoint(ignition::math::Vector3d(1., 1., 1.), ignition::math::Color(0, 0, 1));
    gzdbg << "CableVisualPlugin::Init add 2nd point" << std::endl;
    line_->AddPoint(ignition::math::Vector3d(1., 1., 2.), ignition::math::Color(0, 0, 1));

    gzdbg << "CableVisualPlugin::Init end" << std::endl;
  }

  void CableVisualPlugin::UpdateChild() {
    gzdbg << "CableVisualPlugin::UpdateChild" << std::endl;
    // UpdateRendering();
  }

  void CableVisualPlugin::UpdateVisual(const gazebo_msgs::LinkStatesConstPtr & _msg) {
    gzdbg << *_msg << std::endl;
    // line_->AddPoint(ignition::math::Vector3d(1., 1., 2.), ignition::math::Color(0, 0, 1));
    // line_->SetPoint(1, ignition::math::Vector3d(-1., 1., 3.));
  }

  void CableVisualPlugin::QueueThread(){
    static const double timeout = .01;
    while (node_handle_->ok()){
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

  GZ_REGISTER_VISUAL_PLUGIN(CableVisualPlugin)
}
