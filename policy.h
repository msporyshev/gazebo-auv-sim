#pragma once

#include <string>

#include <msg_regul.h>

#include <regul.pb.h>

#include "transport_pipe.h"


struct RegulConsts {
    const char* IPC_NAME = MSG_REGUL_NAME;
    const char* IPC_FORMAT = MSG_REGUL_FORMAT;
    const std::string TOPIC = "~/regul";
};

struct RegulPolicy {
    typedef MSG_REGUL_TYPE RecieveMsg;
    typedef IPCReciever<RecieveMsg, RegulConsts> RecieverClass;

    typedef msgs::Regul ForwardMsg;
    typedef GazeboForwarder<ForwardMsg, RegulConsts> ForwarderClass;
};