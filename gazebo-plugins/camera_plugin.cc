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
      const unsigned char *data,
      unsigned int width, unsigned int height, unsigned int depth,
      const std::string &format)
  {
    if (this->workingCameraType != this->type) {
      return;
    }
    ::msgs::Camera cameraMsg;

    msgs::Image* frame = cameraMsg.mutable_frame();
    frame->set_data(data, width * height * depth);
    frame->set_width(width);
    frame->set_height(height);
    frame->set_step(width * depth);
    frame->set_pixel_format(common::Image::ConvertPixelFormat(format));

    cameraMsg.set_camera_type(type);

    this->cameraPublisher->Publish(cameraMsg);
  }

  void RecieveSwitchCamera(const boost::shared_ptr<const ::msgs::Camera>& msg) {
    this->workingCameraType = static_cast<::msgs::Camera::CameraType>(msg->camera_type());
  }

private:
  sensors::SensorPtr sensor;
  ::msgs::Camera::CameraType type, workingCameraType = ::msgs::Camera::DOWN;
  transport::NodePtr node;
  transport::PublisherPtr cameraPublisher;
  transport::SubscriberPtr switchCameraSub;

};

GZ_REGISTER_SENSOR_PLUGIN(CameraPublisher)
}