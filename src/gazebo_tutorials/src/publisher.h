#ifndef PUB_HH
#define PUB_HH
#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/math/gzmath.hh"
#include <ros/ros.h>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <std_msgs/Float64.h>
namespace gazebo {
class GAZEBO_VISIBLE distPublisher : public SensorPlugin {
public: distPublisher();
public: virtual ~distPublisher();
public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);
protected: virtual void OnUpdate(sensors::RaySensorPtr _sensor);
protected: sensors::RaySensorPtr parentSensor;
private: event::ConnectionPtr connection;
ros::NodeHandle nh;
ros::Publisher publisher;
};
}
#endif