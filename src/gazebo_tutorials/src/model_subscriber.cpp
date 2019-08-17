/*#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>
#include <ignition/math.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <std_msgs/String.h>

#include <iostream>
#include <vector>
#include <string>

namespace gazebo
{
class WheelPlugin : public ModelPlugin
{
private:
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
    ros::NodeHandle *nh;
    ros::Publisher pub;
    int i;

public:
    WheelPlugin();
    ~WheelPlugin();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    virtual void Update();
    void chatterCallback();
};

void WheelPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    ROS_INFO_STREAM("Initializing Wheel plugin.");
    
    
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "marker_dropper", ros::init_options::NoSigintHandler);
    if(!ros::isInitialized())
    {
        std::cerr << "ROS Not initialized" << std::endl;
        return;
    }

    ROS_INFO_STREAM("ROS Initialized!");
    nh = new ros::NodeHandle();

    ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);

    //world->InsertModelFile("model://marker");


    // Set up Update to be called every simulation update
    // (which is frequently)
    //updateConnection = event::Events::ConnectWorldUpdateBegin(
      //      boost::bind(&WheelPlugin::Update, this));
    
    //ROS_INFO("Marker Dropper Plugin loaded");
}

void WheelPlugin::Update()
{
    this->model->SetLinearVel(ignition::math::Vector3d(.1, 0, 0));

    ROS_INFO_STREAM("Initializing Wheel plugin.");
}

void WheelPlugin::chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  this->model->SetLinearVel(ignition::math::Vector3d(.1, 0, 0));
}

GZ_REGISTER_MODEL_PLUGIN(WheelPlugin)
}
*/