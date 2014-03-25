#pragma once

#include <memory>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/common/common.hh>

#include <msg_robosub.h>
#include <msg_regul.h>
#include <msg_navig.h>

#include <regul.pb.h>
#include <navig.pb.h>
#include <camera.pb.h>

#include <opencv2/opencv.hpp>

#include "ipc_message.h"

msgs::Regul convert(const MSG_REGUL_TYPE& msg) {
    msgs::Regul result;
    *result.mutable_force_ratio() = gazebo::msgs::Convert(gazebo::math::Vector3(msg.tx, msg.ty, msg.tz));
    *result.mutable_torque_ratio() = gazebo::msgs::Convert(gazebo::math::Vector3(msg.mx, msg.my, msg.mz));

    return result;
}

IPCMessage<MSG_NAVIG_TYPE> convert(const msgs::Navig& msg) {
    MSG_NAVIG_TYPE result;

    result.X_KNS = msg.position().x();
    result.Y_KNS = msg.position().y();
    result.Depth_NS = -msg.position().z();

    result.Roll_NS = msg.angle().x();
    result.Psi_NS = msg.angle().y();
    result.Fi_NS = msg.angle().z();

    return make_ipc_msg(result);
}

CameraMessage convert(const msgs::Camera& msg) {
    MSG_JPEG_VIDEO_FRAME m;

    cv::Mat frame(
        msg.frame().height(),
        msg.frame().width(),
        CV_8UC3,
        const_cast<char*>(msg.frame().data().c_str()));

    std::vector<uchar> buff(frame.cols * frame.rows * 3);

    std::vector<int> param = {CV_IMWRITE_JPEG_QUALITY, 100};
    cv::imencode(".jpg", frame, buff, param);

    std::shared_ptr<uchar> dyn_data(new uchar[buff.size()], std::default_delete<uchar[]>());

    m.camera_type = msg.camera_type() == msgs::Camera::FRONT ? CAMERA_FRONT : CAMERA_DOWN;
    m.frame_type = ORIGINAL_FRAME;
    m.size = buff.size();
    m.frame = dyn_data.get();
    memcpy(m.frame, buff.data(), buff.size());

    return make_ipc_msg(m, dyn_data);
}