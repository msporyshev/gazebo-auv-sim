#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include <iostream>
#include <cstdio>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <regul.pb.h>
#include <navig.pb.h>

typedef boost::shared_ptr<const msgs::Regul> RegulPtr;

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
  public:

    void Load(physics::ModelPtr _parent, sdf::ElementPtr)
    {

      this->model = _parent;

      this->node = transport::NodePtr(new transport::Node());

      this->node->Init("robosub_auv");

      this->regulSubscriber = node->Subscribe("~/regul", &ModelPush::UpdateRegul, this);
      gzmsg << "Subscribed to topic: " << regulSubscriber->GetTopic() << std::endl;

      this->navigPublisher = node->Advertise< ::msgs::Navig>("~/navig");
      gzmsg << "Advertised to topic: " << navigPublisher->GetTopic() << std::endl;

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ModelPush::OnUpdate, this, _1));

      this->timer.Start();
    }

    void SendNavig(const math::Pose& pose)
    {
      gzmsg << "Sending navig message" << std::endl;

      ::msgs::Navig msg;
      *msg.mutable_position() = msgs::Convert(pose.pos);
      *msg.mutable_angle() = msgs::Convert(pose.rot.GetAsEuler());
      navigPublisher->Publish(msg);
      this->timer.Stop();
      this->timer.Start();
    }

    void OnUpdate(const common::UpdateInfo & _info)
    {
      this->model->GetLink("link")->SetForce(math::Vector3(0,0,0));
      this->model->GetLink("link")->SetTorque(math::Vector3(0,0,0));
      this->model->GetLink("link")->AddRelativeForce(forceRatio * maxForce);
      this->model->GetLink("link")->AddRelativeTorque(torqueRatio * maxTorque);

      if (this->timer.GetElapsed().Double() >= NAVIG_UPDATE_TIME) {
        SendNavig(this->model->GetLink("link")->GetWorldCoGPose());
      }
    }

    void UpdateRegul(const RegulPtr& msg) {
      gzmsg << "Updating regul" << std::endl;
      this->forceRatio = msgs::Convert(msg->force_ratio());
      this->torqueRatio = msgs::Convert(msg->torque_ratio());
    }

  private:
    static constexpr double NAVIG_UPDATE_TIME = 1;

    transport::NodePtr node;
    transport::SubscriberPtr regulSubscriber;
    transport::PublisherPtr navigPublisher;

    math::Vector3 forceRatio, torqueRatio;
    double maxForce = 3, maxTorque = 3;

    physics::ModelPtr model;

    common::Timer timer;

    event::ConnectionPtr timerConnection;
    event::ConnectionPtr updateConnection;
};

  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
