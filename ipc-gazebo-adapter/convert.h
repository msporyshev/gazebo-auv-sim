#pragma once
#include <gazebo/msgs/msgs.hh>

#include <msg_robosub.h>
#include <msg_regul.h>
#include <msg_navig.h>
#include <msg_compass.h>

#include <regul.pb.h>
#include <navig.pb.h>
#include <camera.pb.h>
#include <compass.pb.h>

#include "ipc_message.h"

template<typename OutputMsg, typename InputMsg>
OutputMsg convert(const InputMsg& msg) {}

template<>
msgs::Regul convert(const MSG_REGUL_TYPE& msg);

template<>
msgs::ipc::Message<MSG_NAVIG_TYPE> convert(const msgs::Navig& msg);

template<>
msgs::ipc::Message<MSG_COMPASS_TYPE> convert(const msgs::Compass& msg);

template<>
msgs::ipc::RawCamera convert(const msgs::Camera& msg);

template<>
msgs::ipc::JpegCamera convert(const msgs::Camera& msg);

template<>
msgs::Camera convert(const MSG_SWITCH_CAMERA& msg);