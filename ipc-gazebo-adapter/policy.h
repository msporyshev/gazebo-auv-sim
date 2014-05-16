#pragma once

#include <string>
#include <map>
#include <memory>

#include <gazebo/msgs/msgs.hh>

#include <msg_regul.h>
#include <msg_robosub.h>
#include <msg_navig.h>
#include <msg_compass.h>

#include <regul.pb.h>
#include <navig.pb.h>
#include <camera.pb.h>
#include <compass.pb.h>

#include "transport_pipe.h"
#include "ipc_message.h"

#define REGISTER_POLICY(Name, PolicyType, RecieveMsg, ForwardMsg, Consts) \
volatile PolicyRegistrator<PolicyType<RecieveMsg, ForwardMsg, Consts> > Name##____(#Name);


class AbstractRegistrator {
public:
    static std::map<std::string, std::unique_ptr<AbstractPipe> > pipe_by_name;
};

template<typename Policy>
class PolicyRegistrator: public AbstractRegistrator {
public:
    PolicyRegistrator(std::string name) {
        pipe_by_name[name] = std::unique_ptr<AbstractPipe>(new TransportPipe<Policy>());
    }
};

template<class RecieveMsgType, class ForwardMsgType, class Consts>
struct IPCToGazeboPolicy {
    typedef RecieveMsgType RecieveMsg;
    typedef IPCReciever<RecieveMsg, Consts> RecieverClass;

    typedef ForwardMsgType ForwardMsg;
    typedef GazeboForwarder<ForwardMsg, Consts> ForwarderClass;
};

template<class RecieveMsgType, class ForwardMsgType, class Consts>
struct GazeboToIPCPolicy {
    typedef RecieveMsgType RecieveMsg;
    typedef GazeboReciever<RecieveMsg, Consts> RecieverClass;

    typedef msgs::ipc::Message<ForwardMsgType> ForwardMsg;
    typedef IPCForwarder<ForwardMsg, Consts> ForwarderClass;
};

struct RegulConsts {
    const char* IPC_NAME = MSG_REGUL_NAME;
    const char* IPC_FORMAT = MSG_REGUL_FORMAT;
    const std::string TOPIC = "~/regul";
};

struct JpegCameraConsts {
    const char* IPC_NAME = MSG_JPEG_VIDEO_FRAME_NAME;
    const char* IPC_FORMAT = MSG_JPEG_VIDEO_FRAME_FORMAT;
    const std::string TOPIC = "~/camera";
};

struct RawCameraConsts {
    const char* IPC_NAME = MSG_VIDEO_FRAME_NAME;
    const char* IPC_FORMAT = MSG_VIDEO_FRAME_FORMAT;
    const std::string TOPIC = "~/camera";
};

struct NavigConsts {
    const char* IPC_NAME = MSG_NAVIG_NAME;
    const char* IPC_FORMAT = MSG_NAVIG_FORMAT;
    const std::string TOPIC = "~/navig";
};

struct SwitchCameraConsts {
    const char* IPC_NAME = MSG_SWITCH_CAMERA_NAME;
    const char* IPC_FORMAT = MSG_SWITCH_CAMERA_FORMAT;
    const std::string TOPIC = "~/switch_camera";
};

struct CompassConsts {
    const char* IPC_NAME = MSG_COMPASS_NAME;
    const char* IPC_FORMAT = MSG_COMPASS_FORMAT;
    const std::string TOPIC = "~/imu";
};
