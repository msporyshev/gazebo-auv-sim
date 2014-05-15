#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include <iostream>
#include <cstdio>
#include <string>
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
#include <gripper.pb.h>

const std::string AUV_MODEL = "auv_body";
const std::string TORPEDO_MODEL = "torpedo";
const std::string LINK = "link";
const std::string JOINT = "joint";

typedef boost::shared_ptr<const msgs::Regul> RegulPtr;
typedef boost::shared_ptr<const msgs::Shoot> MsgShootPtr;
typedef boost::shared_ptr<const msgs::Gripper> MsgGripperPtr;

namespace gazebo
{
  class RobosubPlugin : public WorldPlugin
  {
  public:

    void Load(physics::WorldPtr _parent, sdf::ElementPtr)
    {
      this->world = _parent;

      this->auvBody = _parent->GetModel(AUV_MODEL);
      this->leftTorpedo = this->auvBody->GetLink("left_torpedo::link");
      this->rightTorpedo = this->auvBody->GetLink("right_torpedo::link");

      this->node = transport::NodePtr(new transport::Node());

      this->node->Init("robosub_auv");

      this->regulSubscriber = node->Subscribe("~/regul", &RobosubPlugin::UpdateRegul, this);
      gzmsg << "Subscribed to topic: " << regulSubscriber->GetTopic() << std::endl;

      this->gripperSubscriber = node->Subscribe("~/gripper", &RobosubPlugin::RecieveGripMessage, this);
      gzmsg << "Subscribed to topic: " << gripperSubscriber->GetTopic() << std::endl;

      this->shootSubscriber = node->Subscribe("~/shoot", &RobosubPlugin::RecieveShootMsg, this);
      gzmsg << "Subscribed to topic: " << shootSubscriber->GetTopic() << std::endl;

      this->navigPublisher = node->Advertise< ::msgs::Navig>("~/navig");
      gzmsg << "Advertised to topic: " << navigPublisher->GetTopic() << std::endl;

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&RobosubPlugin::OnUpdate, this, _1));

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

    void StartGripping() {
      auto l_finger = this->auvBody->GetJoint("simple_gripper::palm_left_finger");
      auto r_finger = this->auvBody->GetJoint("simple_gripper::palm_right_finger");
      l_finger->SetForce(0, -1000.5);
      r_finger->SetForce(0, 1000.5);
      // l_finger->SetForce(0, math::Vector3(0, 0.1, 0));
      // r_finger->SetForce(0, math::Vector3(0, -0.1, 0));
    }

    void StopGripping() {
      for (std::string fingername : {"left", "right"}) {
        auto finger = this->auvBody->GetLink(fingername + "_finger");
        finger->Reset();
      }
    }

    void ResetGripper() {
      auto l_finger = this->auvBody->GetJoint("simple_gripper::palm_left_finger");
      auto r_finger = this->auvBody->GetJoint("simple_gripper::palm_right_finger");
      l_finger->SetForce(0, 1000.5);
      r_finger->SetForce(0, -1000.5);
    }

    void RecieveGripMessage(const MsgGripperPtr& msg) {
      switch(msg->action()) {
      case ::msgs::Gripper::START:
        this->StartGripping();
        break;
      case ::msgs::Gripper::STOP:
        this->StopGripping();
        break;
      case ::msgs::Gripper::RESET:
        this->ResetGripper();
        break;
      }
    }

    void Shoot(::msgs::Shoot::TorpedoType type)
    {
      auto link = this->Torpedo(type);
      gzmsg << "Shooting " << ToString(type) << " torpedo" << std::endl;

      this->TorpedoJoint(type)->Detach();
      link->AddRelativeForce(math::Vector3(0, MAX_FORCE / 10, 0));
      link->AddRelativeForce(
            math::Vector3(0, 0, CurBuoyantForce(link->GetWorldCoGPose())));
    }

    void RecieveShootMsg(const MsgShootPtr& msg)
    {
      Shoot(msg->torpedo_type());
    }

    void OnUpdate(const common::UpdateInfo & _info)
    {
      auto link = GetDefaultLink(this->auvBody);
      link->SetForce(
        math::Vector3(0, 0, CurBuoyantForce(link->GetWorldCoGPose())));
      link->SetTorque(math::Vector3(0,0,0));

      link->AddRelativeForce(forceRatio * MAX_FORCE);
      link->AddRelativeTorque(torqueRatio * MAX_TORQUE);

      if (this->timer.GetElapsed().Double() >= NAVIG_UPDATE_TIME)
      {
        SendNavig(link->GetWorldCoGPose());
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
    physics::LinkPtr Torpedo(::msgs::Shoot::TorpedoType type) {
      return (type == ::msgs::Shoot::LEFT ? this->leftTorpedo : this->rightTorpedo);
    }

    physics::JointPtr TorpedoJoint(::msgs::Shoot::TorpedoType type) {
      return (type == ::msgs::Shoot::LEFT ? this->auvBody->GetJoint("left_torpedo_joint") : this->auvBody->GetJoint("right_torpedo_joint"));
    }

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

    static physics::JointPtr GetDefaultJoint(physics::ModelPtr model) {
      return model->GetJoint(JOINT);
    }

    static physics::LinkPtr GetDefaultLink(physics::ModelPtr model) {
      return model->GetLink(LINK);
    }

    static constexpr double NAVIG_UPDATE_TIME = 0.1;

    transport::NodePtr node;
    transport::SubscriberPtr regulSubscriber;
    transport::SubscriberPtr shootSubscriber;
    transport::SubscriberPtr gripperSubscriber;
    transport::PublisherPtr navigPublisher;

    static constexpr double BUOYANT_FORCE = 410;

    math::Vector3 forceRatio, torqueRatio;
    static constexpr double MAX_FORCE = 3, MAX_TORQUE = 3;

    static constexpr double SURFACE_H = 0;
    static constexpr double AUV_H = 1;

    physics::WorldPtr world;

    physics::ModelPtr auvBody;
    physics::LinkPtr leftTorpedo, rightTorpedo;

    bool flag = false;

    common::Timer timer;

    event::ConnectionPtr timerConnection;
    event::ConnectionPtr updateConnection;
  };

  GZ_REGISTER_WORLD_PLUGIN(RobosubPlugin)
}
