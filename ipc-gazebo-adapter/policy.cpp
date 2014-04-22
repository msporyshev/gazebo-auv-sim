#include <memory>
#include <string>
#include <map>
#include <mutex>

#include "policy.h"
#include "transport_pipe.h"


struct RegulConsts {
    const char* IPC_NAME = MSG_REGUL_NAME;
    const char* IPC_FORMAT = MSG_REGUL_FORMAT;
    const std::string TOPIC = "~/regul";
};
REGISTER_POLICY(RegulPolicy, IPCToGazeboPolicy, MSG_REGUL_TYPE, msgs::Regul, RegulConsts);

struct JpegCameraConsts {
    const char* IPC_NAME = MSG_JPEG_VIDEO_FRAME_NAME;
    const char* IPC_FORMAT = MSG_JPEG_VIDEO_FRAME_FORMAT;
    const std::string TOPIC = "~/camera";
};
REGISTER_POLICY(JpegCameraPolicy, GazeboToIPCPolicy, msgs::Camera, MSG_JPEG_VIDEO_FRAME, JpegCameraConsts);

struct RawCameraConsts {
    const char* IPC_NAME = MSG_VIDEO_FRAME_NAME;
    const char* IPC_FORMAT = MSG_VIDEO_FRAME_FORMAT;
    const std::string TOPIC = "~/camera";
};
REGISTER_POLICY(RawCameraPolicy, GazeboToIPCPolicy, msgs::Camera, MSG_VIDEO_FRAME, RawCameraConsts);

struct NavigConsts {
    const char* IPC_NAME = MSG_NAVIG_NAME;
    const char* IPC_FORMAT = MSG_NAVIG_FORMAT;
    const std::string TOPIC = "~/navig";
};
REGISTER_POLICY(NavigPolicy, GazeboToIPCPolicy, msgs::Navig, MSG_NAVIG_TYPE, NavigConsts);

struct SwitchCameraConsts {
    const char* IPC_NAME = MSG_SWITCH_CAMERA_NAME;
    const char* IPC_FORMAT = MSG_SWITCH_CAMERA_FORMAT;
    const std::string TOPIC = "~/switch_camera";
};
REGISTER_POLICY(SwitchCameraPolicy, IPCToGazeboPolicy, MSG_SWITCH_CAMERA, msgs::Camera, SwitchCameraConsts);

struct CompassConsts {
    const char* IPC_NAME = MSG_COMPASS_NAME;
    const char* IPC_FORMAT = MSG_COMPASS_FORMAT;
    const std::string TOPIC = "~/imu";
};
REGISTER_POLICY(compass, GazeboToIPCPolicy, msgs::Compass, MSG_COMPASS_TYPE, CompassConsts)