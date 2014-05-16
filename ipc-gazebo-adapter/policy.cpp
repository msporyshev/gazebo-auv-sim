#include <memory>
#include <string>
#include <map>
#include <mutex>

#include "policy.h"
#include "transport_pipe.h"

std::map<std::string, std::unique_ptr<AbstractPipe> > AbstractRegistrator::pipe_by_name;
std::mutex global_mutex;

REGISTER_POLICY(RegulPolicy, IPCToGazeboPolicy, MSG_REGUL_TYPE, msgs::Regul, RegulConsts)

REGISTER_POLICY(JpegCameraPolicy, GazeboToIPCPolicy, msgs::Camera, MSG_JPEG_VIDEO_FRAME, JpegCameraConsts)

REGISTER_POLICY(RawCameraPolicy, GazeboToIPCPolicy, msgs::Camera, MSG_VIDEO_FRAME, RawCameraConsts)

REGISTER_POLICY(NavigPolicy, GazeboToIPCPolicy, msgs::Navig, MSG_NAVIG_TYPE, NavigConsts)

REGISTER_POLICY(SwitchCameraPolicy, IPCToGazeboPolicy, MSG_SWITCH_CAMERA, msgs::Camera, SwitchCameraConsts)

REGISTER_POLICY(CompassPolicy, GazeboToIPCPolicy, msgs::Compass, MSG_COMPASS_TYPE, CompassConsts)