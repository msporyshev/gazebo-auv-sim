#pragma once

#include <msg_robosub.h>
#include <memory>

namespace msgs {
namespace ipc {

template<typename MsgType, typename DynData = uchar>
struct Message {
    MsgType msg;
    std::shared_ptr<DynData> dynamic_data;
    Message(MsgType msg, std::shared_ptr<DynData> data = nullptr): msg(msg), dynamic_data(data) { }
};

template<typename MsgType, typename DynData = uchar>
Message<MsgType, DynData> make_msg(const MsgType& msg, std::shared_ptr<DynData> data = nullptr) {
    return Message<MsgType, DynData>(msg, data);
}

using JpegCamera = Message<MSG_JPEG_VIDEO_FRAME, uchar>;
using RawCamera = Message<MSG_VIDEO_FRAME, uchar>;

} // namespace ipc
} // namespace msgs