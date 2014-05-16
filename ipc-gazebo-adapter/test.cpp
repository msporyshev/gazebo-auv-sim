#include <gazebo/transport/transport.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <iostream>
#include <string>
#include <memory>
#include <cerrno>
#include <mutex>

#include <ipc.h>

#include "exception.h"
#include "common.h"
#include "policy.h"
#include "transport_pipe.h"

#include <future>
#include <functional>

#include <regul.pb.h>
#include <camera.pb.h>
#include <navig.pb.h>

#define BOOST_TEST_MODULE AdapterTests
#include <boost/test/unit_test.hpp>

template<typename Msg>
using MsgPtr = boost::shared_ptr<const Msg>;

template<typename Consts>
struct Fixture {
    Fixture() {
        IPC_connectModule("adapter tester", "localhost");
        IPC_defineMsg(consts_.IPC_NAME, IPC_VARIABLE_LENGTH, consts_.IPC_FORMAT);

        gazebo::setupClient(0, nullptr);

        node = gazebo::transport::NodePtr(new gazebo::transport::Node());
        node->Init("robosub_auv");
    }

    virtual ~Fixture() {
        gazebo::shutdown();
        IPC_disconnect();
    }

    gazebo::transport::NodePtr node;

    bool ready = false;
    Consts consts_;
};


template<typename GazeboMsg, typename IpcMsg, typename Consts>
struct IGFixture: public Fixture<Consts> {
    IGFixture(): Fixture<Consts>() {}

    GazeboMsg gazebo_msg;
    gazebo::transport::SubscriberPtr subscriber;

    void publish(IpcMsg& msg) {
        IPC_publishData(this->consts_.IPC_NAME, &msg);
    }

    void subscribe(std::string topic) {
        subscriber = this->node->Subscribe(topic, &IGFixture::callback, static_cast<IGFixture*>(this));
        gazebo::common::Time::MSleep(100);
    }

    void callback(const boost::shared_ptr<const GazeboMsg>& _msg) {
        std::cout << "RECIEVED" << std::endl;
        this->gazebo_msg = *_msg;
        this->ready = true;
    }
};

template<typename IpcMsg, typename GazeboMsg, typename Consts>
struct GIFixture: public Fixture<Consts> {
    GIFixture(): Fixture<Consts>() {
        IPC_subscribeData(Consts().IPC_NAME, callback, this);
    }

    IpcMsg ipc_msg;
    gazebo::transport::PublisherPtr publisher;

    void publish(const GazeboMsg& msg) {
        publisher->Publish(msg);
    }

    void advertise(std::string topic) {
        this->publisher = this->node->template Advertise<GazeboMsg>(topic);
        this->publisher->WaitForConnection();
    }

    static void callback(MSG_INSTANCE msgInstance, void *callData, void* clientData) {
        std::cout << "RECIEVED" << std::endl;
        auto client = static_cast<GIFixture*>(clientData);

        client->ipc_msg = *static_cast<IpcMsg*>(callData);
        client->ready = true;
    }
};

using RegulFixture = IGFixture<msgs::Regul, MSG_REGUL_TYPE, RegulConsts>;
using SwitchCameraFixture = IGFixture<msgs::Camera, MSG_SWITCH_CAMERA, SwitchCameraConsts>;
using JpegCameraFixture = GIFixture<MSG_JPEG_VIDEO_FRAME, msgs::Camera, JpegCameraConsts>;
using RawCameraFixture = GIFixture<MSG_VIDEO_FRAME, msgs::Camera, RawCameraConsts>;
using NavigFixture = GIFixture<MSG_NAVIG_TYPE, msgs::Navig, NavigConsts>;
using CompassFixture = GIFixture<MSG_COMPASS_TYPE, msgs::Compass, CompassConsts>;

BOOST_FIXTURE_TEST_CASE(regul_pipe_test, RegulFixture) {
    try {
        subscribe("~/regul");

        MSG_REGUL_TYPE msg;
        msg.tx = 1;
        msg.ty = 2;
        msg.tz = 3;
        msg.mx = 4;
        msg.my = 5;
        msg.mz = 6;

        publish(msg);

        while (!ready) {
            IPC_listenClear(100);
            gazebo::common::Time::MSleep(100);
        }

        BOOST_CHECK_EQUAL(msg.tx, gazebo_msg.force_ratio().x());
        BOOST_CHECK_EQUAL(msg.ty, gazebo_msg.force_ratio().y());
        BOOST_CHECK_EQUAL(msg.tz, gazebo_msg.force_ratio().z());
        BOOST_CHECK_EQUAL(msg.mx, -gazebo_msg.torque_ratio().x());
        BOOST_CHECK_EQUAL(msg.my, -gazebo_msg.torque_ratio().y());
        BOOST_CHECK_EQUAL(msg.mz, -gazebo_msg.torque_ratio().z());
    } catch (Exception& e) {
        FATAL() << e;
    }
}

BOOST_FIXTURE_TEST_CASE(switch_camera_pipe_test, SwitchCameraFixture) {
    try {
        subscribe("~/switch_camera");


        MSG_SWITCH_CAMERA msg;
        msg.camera_type = CAMERA_DOWN;
        std::vector<unsigned char> r = {RK_BUOY, RK_BLACK_STRIPE, RK_BALL};
        msg.recognizers = r.data();
        msg.recognizers_size = r.size();

        publish(msg);

        while (!ready) {
            IPC_listenClear(100);
            gazebo::common::Time::MSleep(100);
        }

        BOOST_CHECK_EQUAL(msg.camera_type, static_cast<unsigned char>(gazebo_msg.camera_type()));
    } catch (Exception& e) {
        FATAL() << e;
    }
}

BOOST_FIXTURE_TEST_CASE(compass_pipe_test, CompassFixture) {
    try {
        advertise("~/imu");

        msgs::Compass msg;

        msg.set_time(123123.123123);
        Set(msg.mutable_orientation(), gazebo::math::Vector3(1,2,3));
        Set(msg.mutable_angular_vel(), gazebo::math::Vector3(4,5,6));
        Set(msg.mutable_linear_accel(), gazebo::math::Vector3(7,8,9));

        gazebo::common::Time::MSleep(100);

        publish(msg);

        while (!ready) {
            IPC_listenClear(100);
            gazebo::common::Time::MSleep(100);
        }

        BOOST_CHECK_EQUAL(ipc_msg.time, msg.time());

        BOOST_CHECK_EQUAL(ipc_msg.state, 0);

        BOOST_CHECK_EQUAL(ipc_msg.roll, msg.orientation().x());
        BOOST_CHECK_EQUAL(ipc_msg.pitch, msg.orientation().y());
        BOOST_CHECK_EQUAL(ipc_msg.heading, msg.orientation().z());

        BOOST_CHECK_EQUAL(ipc_msg.roll_rate, msg.angular_vel().x());
        BOOST_CHECK_EQUAL(ipc_msg.pitch_rate, msg.angular_vel().y());
        BOOST_CHECK_EQUAL(ipc_msg.head_rate, msg.angular_vel().z());

        BOOST_CHECK_EQUAL(ipc_msg.accX, msg.linear_accel().x());
        BOOST_CHECK_EQUAL(ipc_msg.accY, msg.linear_accel().y());
        BOOST_CHECK_EQUAL(ipc_msg.accZ, msg.linear_accel().z());

    } catch (Exception& e) {
        FATAL() << e;
    }
}

BOOST_FIXTURE_TEST_CASE(navig_pipe_test, NavigFixture) {
    try {
        advertise("~/navig");

        msgs::Navig msg;
        auto pos = gazebo::math::Vector3(1,2,3);
        auto ang = gazebo::math::Vector3(4,5,6);

        Set(msg.mutable_position(), pos);
        Set(msg.mutable_angle(), ang);

        publish(msg);

        while (!ready) {
            IPC_listenClear(100);
            gazebo::common::Time::MSleep(100);
        }

        BOOST_CHECK_EQUAL(ipc_msg.X_KNS, msg.position().x());
        BOOST_CHECK_EQUAL(ipc_msg.Y_KNS, msg.position().y());
        BOOST_CHECK_EQUAL(ipc_msg.Depth_NS, -msg.position().z());

        BOOST_CHECK_EQUAL(ipc_msg.Roll_NS, msg.angle().x());
        BOOST_CHECK_EQUAL(ipc_msg.Psi_NS, msg.angle().y());
        BOOST_CHECK_EQUAL(ipc_msg.Fi_NS, msg.angle().z());
    } catch (Exception& e) {
        FATAL() << e;
    }
}
