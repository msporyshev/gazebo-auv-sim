#pragma once

#include <gazebo/msgs/msgs.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/common/common.hh>

#include <msg_robosub.h>
#include <msg_regul.h>

#include "regul.pb.h"

msgs::Regul convert(const MSG_REGUL_TYPE& msg) {
    msgs::Regul result;
    *result.mutable_force_ratio() = gazebo::msgs::Convert(gazebo::math::Vector3(msg.tx, msg.ty, msg.tz));
    *result.mutable_torque_ratio() = gazebo::msgs::Convert(gazebo::math::Vector3(msg.mx, msg.my, msg.mz));

    return result;
}