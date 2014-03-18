#pragma once

#include <memory>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/common/common.hh>

#include <msg_robosub.h>
#include <msg_regul.h>
#include <msg_navig.h>

#include "regul.pb.h"
#include "navig.pb.h"
#include "camera.pb.h"


msgs::Regul convert(const MSG_REGUL_TYPE& msg) {
    msgs::Regul result;
    *result.mutable_force_ratio() = gazebo::msgs::Convert(gazebo::math::Vector3(msg.tx, msg.ty, msg.tz));
    *result.mutable_torque_ratio() = gazebo::msgs::Convert(gazebo::math::Vector3(msg.mx, msg.my, msg.mz));

    return result;
}

MSG_NAVIG_TYPE convert(const msgs::Navig& msg) {
    MSG_NAVIG_TYPE result;

    result.X_KNS = msg.position().x();
    result.Y_KNS = msg.position().y();
    result.Depth_NS = -msg.position().z();

    result.Fi_NS = msg.angle().x();
    result.Psi_NS = msg.angle().y();
    result.Roll_NS = msg.angle().z();

    return result;
}

MSG_VIDEO_FRAME convert(const msgs::Camera& msg) {
    MSG_VIDEO_FRAME m;

    m.camera_type = msg.camera_type() == msgs::Camera::FRONT ? CAMERA_FRONT : CAMERA_DOWN;
    m.w = msg.frame().width();
    m.h = msg.frame().height();
    m.channels = 3;
    m.size = m.w * m.h * m.channels;
    m.frame = malloc(msg.frame().data().size());
    memcpy(m.frame, msg.frame().data().c_str(), msg.frame().data().size());

    return m;
}