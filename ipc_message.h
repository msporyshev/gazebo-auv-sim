#pragma once

#include <msg_robosub.h>
#include <memory>

template<typename MsgType, typename DynData = uchar>
struct IPCMessage {
    MsgType msg;
    std::shared_ptr<DynData> dynamic_data;
    IPCMessage(MsgType msg, std::shared_ptr<DynData> data = nullptr): msg(msg), dynamic_data(data) { }
};

template<typename MsgType, typename DynData = uchar>
IPCMessage<MsgType, DynData> make_ipc_msg(const MsgType& msg, std::shared_ptr<DynData> data = nullptr) {
    return IPCMessage<MsgType, DynData>(msg, data);
}

using CameraMessage = IPCMessage<MSG_JPEG_VIDEO_FRAME, uchar>;