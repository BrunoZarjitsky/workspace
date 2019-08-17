#include "publisher.h"
namespace gazebo{
distPublisher::distPublisher()//:
//nh("robot")
{
publisher = nh.advertise<std_msgs::Float64>("distance", 1);
}
distPublisher::~distPublisher() {
this->parentSensor->DisconnectUpdated(this->connection);
this->parentSensor.reset();
}
void distPublisher::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {
this->parentSensor = std::dynamic_pointer_cast<sensors::RaySensor>(_parent);
this->connection = this->parentSensor->ConnectUpdated(
std::bind(&distPublisher::OnUpdate, this, this->parentSensor));
}
void distPublisher::OnUpdate(sensors::RaySensorPtr _sensor) {
double min = _sensor->RangeMax();
_sensor->SetActive(false);
for (int i=0;i<_sensor->RangeCount();i++){
if (_sensor->Range(i)<min)
min=_sensor->Range(i);
}
_sensor->SetActive(true);
std_msgs::Float64 msg;
msg.data = min;
publisher.publish(msg);
}
GZ_REGISTER_SENSOR_PLUGIN(distPublisher)
}