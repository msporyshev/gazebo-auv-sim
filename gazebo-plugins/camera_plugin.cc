#include <gazebo/gazebo.hh>
#include <gazebo/plugins/CameraPlugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

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

    this->node = transport::NodePtr(new transport::Node());

    this->node->Init("robosub_auv");

    this->cameraPublisher = node->Advertise< ::msgs::Camera>("~/camera");
    gzmsg << "Advertised to topic: " << cameraPublisher->GetTopic();
  }

  public: void OnNewFrame(
      const unsigned char *_image,
      unsigned int _width, unsigned int _height, unsigned int _depth,
      const std::string &_format)
  {
    common::Image image;
    image.SetFromData(_image, _width, _height, common::Image::ConvertPixelFormat(_format));

    ::msgs::Camera cameraMsg;
    msgs::Set(cameraMsg.mutable_frame(), image);

    cameraMsg.set_camera_type(::msgs::Camera::FRONT);

    this->cameraPublisher->Publish(cameraMsg);
  }

private:
  transport::NodePtr node;
  transport::PublisherPtr cameraPublisher;
};

GZ_REGISTER_SENSOR_PLUGIN(CameraPublisher)
}