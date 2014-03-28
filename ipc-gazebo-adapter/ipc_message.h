#pragma once

#include <ipc.h>
#include <msg_robosub.h>
#include <memory>

namespace msgs {
namespace ipc {

template<typename MsgType>
struct Message {
    MsgType msg;
    std::shared_ptr<MsgType> dynamic_data;

    Message(MsgType msg, FORMATTER_PTR formatter)
        : msg(msg)
        , dynamic_data(&this->msg, [formatter](MsgType* ptr) {
            IPC_freeDataElements(formatter, ptr);
        })
    { }
};

template<typename MsgType>
Message<MsgType> make_msg(const MsgType& msg, FORMATTER_PTR formatter) {
    return Message<MsgType>(msg, formatter);
}

template<typename MsgType>
Message<MsgType> make_msg(const MsgType& msg, const char* format) {
    return Message<MsgType>(msg, IPC_parseFormat(format));
}

using JpegCamera = Message<MSG_JPEG_VIDEO_FRAME>;
using RawCamera = Message<MSG_VIDEO_FRAME>;

} // namespace ipc
} // namespace msgs