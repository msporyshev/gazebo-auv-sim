#pragma once
#include <functional>

#include <msg_robosub.h>
#include <msg_regul.h>
#include <msg_navig.h>

#include <ipc.h>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

template<typename MsgType>
using Callback = std::function<void (const MsgType&)>;

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
        MsgConsts consts;
        IPC_defineMsg(consts.IPC_NAME, IPC_VARIABLE_LENGTH, consts.IPC_FORMAT);
        msg_format_ = IPC_parseFormat(consts.IPC_FORMAT);

        IPC_subscribeData(MSG_REGUL_NAME, recieve_msg, this);
    }

    static void recieve_msg(MSG_INSTANCE msgRef, void *callData, void* clientData) {
        auto client = static_cast<const IPCReciever<MsgType, MsgConsts>*>(clientData);

        MsgType *m;
        IPC_unmarshall(client->msg_format_, callData, (void **)&m);

        client->callback_(*m);

        IPC_freeByteArray(callData);
        IPC_freeData(client->msg_format_, m);
    }

    FORMATTER_PTR msg_format_;
};

template<typename MsgType, typename MsgConsts>
class GazeboReciever: public Reciever<MsgType> {
public:
    explicit GazeboReciever(Callback<MsgType> callback, gazebo::transport::Node& node)
            : Reciever<MsgType>(callback), node_(node)
    {
        gazebo::transport::SubscriberPtr sub = node.Subscribe(MsgConsts().TOPIC, &recieve_msg, this);
    }

    void recieve_msg(MsgType msg) {
        callback_(msg);
    }
private:
    gazebo::transport::Node& node_;
};

template<typename MsgType, typename MsgConsts>
class Forwarder {};


struct RegulPipeConsts {
    const char* IPC_NAME = MSG_REGUL_NAME;
    const char* IPC_FORMAT = MSG_REGUL_FORMAT;
    const std::string TOPIC = "~/regul";
};

struct RegulPipeTag {
    typedef MSG_REGUL_TYPE RecieveMsg;
    typedef IPCReciever<RecieveMsg, RegulPipeConsts> RecieverClass;

    typedef gazebo::msgs::Vector3d ForwardMsg;
    typedef Forwarder<ForwardMsg, RegulPipeConsts> ForwarderClass;
};



template<typename PipeTag>
class TransportPipe {
public:
    TransportPipe(gazebo::transport::Node& node)
        : reciever_([&](const typename PipeTag::RecieveMsg& msg) {this->on_recieve(msg);}) { }

    void on_recieve(const typename PipeTag::RecieveMsg& msg) {

    }

private:
    FORMATTER_PTR ipc_msg_format_;
    typename PipeTag::RecieverClass reciever_;
    typename PipeTag::ForwarderClass forwarder_;
};