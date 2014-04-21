#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/math/Vector3.hh>
#include <iostream>
#include <compass.pb.h>

namespace gazebo
{

class ImuPublisher: public SensorPlugin
{
public:
    ImuPublisher() {}

    virtual void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
    {

        this->node = transport::NodePtr(new transport::Node());

        this->node->Init("robosub_auv");

        if (!sensor)
        {
            gzerr << "Invalid sensor pointer." << std::endl;
        }

        this->parentSensor =
            boost::dynamic_pointer_cast<sensors::ImuSensor>(sensor);

        if (!this->parentSensor)
        {
            gzerr << "IMU plugin requires IMU sensor" << std::endl;
        }

        this->imuPublisher = this->node->Advertise<::msgs::Compass>("~/imu");
        this->imuConnection = this->parentSensor->ConnectUpdated(boost::bind(&ImuPublisher::OnUpdate, this));
    }

    void OnUpdate()
    {
        ::msgs::Compass msg;
        auto accel = parentSensor->GetLinearAcceleration();
        auto vel = parentSensor->GetAngularVelocity();
        auto orientation = parentSensor->GetOrientation().GetAsEuler();
        auto time = parentSensor->GetLastMeasurementTime().Double();
        *msg.mutable_linear_accel() = msgs::Convert(accel);
        *msg.mutable_angular_vel() = msgs::Convert(vel);
        *msg.mutable_orientation() = msgs::Convert(orientation);
        msg.set_time(time);

        this->imuPublisher->Publish(msg);
    }

private:
    sensors::ImuSensorPtr parentSensor;
    event::ConnectionPtr imuConnection;
    transport::NodePtr node;
    transport::PublisherPtr imuPublisher;
};

GZ_REGISTER_SENSOR_PLUGIN(ImuPublisher)

}