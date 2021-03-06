#pragma once
#include <functional>
#include <mutex>
#include <cstdio>
#include <string>
#include <cerrno>
#include <memory>

#include <ipc.h>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <boost/shared_ptr.hpp>

#include "exception.h"
#include "common.h"
#include "convert.h"
#include "ipc_message.h"

extern std::mutex global_mutex;

class AbstractPipe {
public:
    virtual void run(gazebo::transport::NodePtr node) = 0;
    virtual void stop() = 0;
    virtual bool is_running() = 0;
};

template<typename MsgType>
using Callback = std::function<void (const MsgType&)>;

template<typename MsgType>
using  MsgPtr = boost::shared_ptr<const MsgType>;

template<typename MsgType>
class Reciever {
public:
    explicit Reciever(Callback<MsgType> callback)
        : callback_(callback) {}

    virtual ~Reciever() {}

protected:
    Callback<MsgType> callback_;
};

template<typename MsgType, typename MsgConsts>
class IPCReciever: public Reciever<MsgType> {
public:
    explicit IPCReciever(Callback<MsgType> callback, gazebo::transport::NodePtr node = nullptr)
            : Reciever<MsgType>(callback)
    {
        if (IPC_defineMsg(consts_.IPC_NAME, IPC_VARIABLE_LENGTH, consts_.IPC_FORMAT) != IPC_OK){
            THROW(Exception(errno, "Unable to define message of type: " + std::string(consts_.IPC_NAME)));
        }

        INFO() << "Subscribing to ipc message: " << consts_.IPC_NAME;
        if (IPC_subscribeData(consts_.IPC_NAME, recieve_msg, this) != IPC_OK) {
            THROW(Exception(errno, "Unable to subscribe message of type: " + std::string(consts_.IPC_NAME)));
        }
    }

    static void recieve_msg(MSG_INSTANCE msgInstance, void *callData, void* clientData) {
        std::lock_guard<std::mutex> lock(global_mutex);
        auto client = static_cast<const IPCReciever<MsgType, MsgConsts>*>(clientData);

        DEBUG() << "Recieved message of type: " << client->consts_.IPC_NAME;
        auto m = static_cast<MsgType*>(callData);

        SCOPE_EXIT {
            IPC_freeData(IPC_msgInstanceFormatter(msgInstance), m);
        };

        client->callback_(*m);
    }

    MsgConsts consts_;
private:
};

template<typename MsgType, typename MsgConsts>
class GazeboReciever: public Reciever<MsgType> {
public:
    explicit GazeboReciever(Callback<MsgType> callback, gazebo::transport::NodePtr node)
            : Reciever<MsgType>(callback), node_(node)
    {
        subscriber_ = node->Subscribe(consts_.TOPIC, &GazeboReciever::recieve_msg, this);
        INFO() << "Subscribed to gazebo topic: " << subscriber_->GetTopic();


        INFO() << "Waiting for connection";
        // gazebo::common::Time::MSleep(100);
        INFO() << SUCCESS;
    }

    void recieve_msg(const MsgPtr<MsgType>& msg) {
        std::lock_guard<std::mutex> lock(global_mutex);
        DEBUG() << "Recieved message from gazebo topic: " << subscriber_->GetTopic();
        this->callback_(*msg);
    }
private:
    gazebo::transport::NodePtr node_;
    gazebo::transport::SubscriberPtr subscriber_;
    MsgConsts consts_;
};

template<typename MsgType, typename MsgConsts>
class GazeboForwarder {
public:
    GazeboForwarder(gazebo::transport::NodePtr node) {
        publisher_ = node->Advertise<MsgType>(consts_.TOPIC);
        INFO() << "Advertised to gazebo topic" << publisher_->GetTopic();

        INFO() << "Waiting for connection";
        // gazebo::common::Time::MSleep(100);
        INFO() << SUCCESS;
    }

    void forward_msg(const MsgType& msg) {
        publisher_->Publish(msg);
        DEBUG() << "Forwarding message: " << std::endl << msg.DebugString();
    }

private:
    gazebo::transport::PublisherPtr publisher_;
    MsgConsts consts_;
};

template<typename MsgType, typename MsgConsts>
class IPCForwarder {
public:
    IPCForwarder(gazebo::transport::NodePtr node = nullptr) {
        IPC_defineMsg(consts_.IPC_NAME, IPC_VARIABLE_LENGTH, consts_.IPC_FORMAT);
    }

    void forward_msg(const MsgType& msg) {
        DEBUG() << "Forwarding message to ipc: " << consts_.IPC_NAME;
        auto fwd = msg.msg;
        IPC_publishData(consts_.IPC_NAME, &fwd);
    }
private:
    MsgConsts consts_;
};

template<typename PipePolicy>
class TransportPipe: public AbstractPipe {
public:
    virtual void run(gazebo::transport::NodePtr node) override {
        reciever_ = std::make_shared<typename PipePolicy::RecieverClass>(
            [&](const typename PipePolicy::RecieveMsg& msg) {this->on_recieve(msg);}, node);

        forwarder_ = std::make_shared<typename PipePolicy::ForwarderClass>(node);
    }

    virtual bool is_running() override {
        return forwarder_ && reciever_;
    }

    virtual void stop() override {
        reciever_.reset();
        forwarder_.reset();
    }

    void on_recieve(const typename PipePolicy::RecieveMsg& msg) {
        forwarder_->forward_msg(convert<typename PipePolicy::ForwardMsg>(msg));
    }

private:
    std::shared_ptr<typename PipePolicy::RecieverClass> reciever_;
    std::shared_ptr<typename PipePolicy::ForwarderClass> forwarder_;
};