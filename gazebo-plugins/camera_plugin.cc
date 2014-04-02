#include <gazebo/gazebo.hh>
#include <gazebo/plugins/CameraPlugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <boost/shared_ptr.hpp>

#include <camera.pb.h>

namespace gazebo
{
class CameraPublisher: public CameraPlugin
{
public:
  CameraPublisher(): CameraPlugin() {}

  void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
  {
    CameraPlugin::Load(_parent, _sdf);
    this->sensor = _parent;
    this->type = _parent->GetName() == "front_camera" ? ::msgs::Camera::FRONT : ::msgs::Camera::DOWN;

    this->node = transport::NodePtr(new transport::Node());

    this->node->Init("robosub_auv");

    this->cameraPublisher = node->Advertise< ::msgs::Camera>("~/camera");
    gzmsg << "Advertised to topic: " << cameraPublisher->GetTopic();

    this->switchCameraSub = node->Subscribe<::msgs::Camera>(
      "~/switch_camera",
      &CameraPublisher::RecieveSwitchCamera, this);
    gzmsg << "Subscribed to topic: " << this->switchCameraSub->GetTopic();
  }

  void OnNewFrame(
      const unsigned char *_image,
      unsigned int _width, unsigned int _height, unsigned int _depth,
      const std::string &_format)
  {
    if (this->workingCameraType != this->type) {
      return;
    }

    common::Image image;
    image.SetFromData(_image, _width, _height, common::Image::ConvertPixelFormat(_format));

    ::msgs::Camera cameraMsg;
    msgs::Set(cameraMsg.mutable_frame(), image);

    cameraMsg.set_camera_type(type);

    this->cameraPublisher->Publish(cameraMsg);
  }

  void RecieveSwitchCamera(const boost::shared_ptr<const ::msgs::Camera>& msg) {
    this->workingCameraType = msg->camera_type();
  }

private:
  sensors::SensorPtr sensor;
  ::msgs::Camera::CameraType type, workingCameraType = ::msgs::Camera::FRONT;
  transport::NodePtr node;
  transport::PublisherPtr cameraPublisher;
  transport::SubscriberPtr switchCameraSub;
};

GZ_REGISTER_SENSOR_PLUGIN(CameraPublisher)
}