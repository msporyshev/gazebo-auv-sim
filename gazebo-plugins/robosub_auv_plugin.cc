#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include <iostream>
#include <cstdio>
#include <cmath>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <regul.pb.h>
#include <navig.pb.h>
#include <shoot.pb.h>

const std::string AUV_LINK = "auv_body_link";
const std::string TORPEDO_LINK = "torpedo_link";

typedef boost::shared_ptr<const msgs::Regul> RegulPtr;
typedef boost::shared_ptr<const msgs::Shoot> MsgShootPtr;

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

      this->shootSubscriber = node->Subscribe("~/shoot", &ModelPush::RecieveShootMsg, this);
      gzmsg << "Subscribed to topic: " << shootSubscriber->GetTopic() << std::endl;

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

    void Shoot(::msgs::Shoot::TorpedoType type)
    {
      auto link = this->model->GetLink(TorpedoLinkName(type));
      gzmsg << "Shooting " << ToString(type) << " torpedo" << std::endl;

      this->model->GetJoint(TorpedoJointName(type))->Detach();
      link->AddRelativeForce(math::Vector3(0, MAX_FORCE * 3, 0));
      link->AddRelativeForce(
            math::Vector3(0, 0, CurBuoyantForce(link->GetWorldCoGPose())));
    }

    void RecieveShootMsg(const MsgShootPtr& msg)
    {
      Shoot(msg->torpedo_type());
    }

    void OnUpdate(const common::UpdateInfo & _info)
    {
      this->model->GetLink(AUV_LINK)->SetForce(math::Vector3(0,0,0));
      this->model->GetLink(AUV_LINK)->SetTorque(math::Vector3(0,0,0));

      this->model->GetLink(AUV_LINK)->AddRelativeForce(
        math::Vector3(0, 0, CurBuoyantForce(this->model->GetLink(AUV_LINK)->GetWorldCoGPose())));
      this->model->GetLink(AUV_LINK)->AddRelativeForce(forceRatio * MAX_FORCE);
      this->model->GetLink(AUV_LINK)->AddRelativeTorque(torqueRatio * MAX_TORQUE);

      if (this->timer.GetElapsed().Double() >= NAVIG_UPDATE_TIME)
      {
        SendNavig(this->model->GetLink(AUV_LINK)->GetWorldCoGPose());
      }

      // Shoot(::msgs::Shoot::RIGHT); // Раскоментировать для демонстрации :D
    }

    void UpdateRegul(const RegulPtr& msg)
    {
      gzmsg << "Updating regul" << std::endl;
      this->forceRatio = msgs::Convert(msg->force_ratio());
      this->torqueRatio = msgs::Convert(msg->torque_ratio());
    }

  private:

    static double CurBuoyantForce(const math::Pose& pose)
    {
      if (pose.pos.z > SURFACE_H)
      {
        double ratio = (AUV_H - (pose.pos.z - SURFACE_H)) / AUV_H;
        ratio = fmax(ratio, 0);

        return BUOYANT_FORCE * ratio;
      } else {
        return BUOYANT_FORCE;
      }
    }

    static std::string ToString(::msgs::Shoot::TorpedoType type)
    {
      return type == ::msgs::Shoot::LEFT ? "left" : "right";
    }

    static std::string TorpedoLinkName(::msgs::Shoot::TorpedoType type)
    {
      return ToString(type) + "_" + TORPEDO_LINK;
    }

    static std::string TorpedoJointName(::msgs::Shoot::TorpedoType type)
    {
      return ToString(type) + "_torpedo_joint";
    }

    static constexpr double NAVIG_UPDATE_TIME = 0.1;

    transport::NodePtr node;
    transport::SubscriberPtr regulSubscriber;
    transport::SubscriberPtr shootSubscriber;
    transport::PublisherPtr navigPublisher;

    static constexpr double BUOYANT_FORCE = 410;

    math::Vector3 forceRatio, torqueRatio;
    static constexpr double MAX_FORCE = 3, MAX_TORQUE = 3;

    static constexpr double SURFACE_H = 0;
    static constexpr double AUV_H = 1;

    physics::ModelPtr model;

    common::Timer timer;

    event::ConnectionPtr timerConnection;
    event::ConnectionPtr updateConnection;
  };

  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
