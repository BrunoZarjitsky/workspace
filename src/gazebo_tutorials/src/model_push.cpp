#include <boost/bind.hpp>

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
};

void WheelPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    ROS_INFO_STREAM("Initializing Wheel plugin.");

    // Set up Update to be called every simulation update
    // (which is frequently)
    updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&WheelPlugin::Update, this));
}

void WheelPlugin::Update()
{
    this->model->SetLinearVel(ignition::math::Vector3d(.1, 0, 0));

    ROS_INFO_STREAM("Initializing Wheel plugin.");
}

GZ_REGISTER_MODEL_PLUGIN(WheelPlugin)
}
