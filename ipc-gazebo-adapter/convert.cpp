#include <memory>

#include <gazebo/math/Vector3.hh>
#include <gazebo/common/common.hh>

#include <opencv2/opencv.hpp>

#include "convert.h"



namespace {

cv::Mat frame_from_msg(const msgs::Camera& msg) {
    cv::Mat frame(
        msg.frame().height(),
        msg.frame().width(),
        CV_8UC3,
        const_cast<char*>(msg.frame().data().c_str()));

    cv::cvtColor(frame, frame, CV_RGB2BGR);

    return frame;
}

} // namespace

template<>
msgs::Regul convert(const MSG_REGUL_TYPE& msg) {
    msgs::Regul result;
    *result.mutable_force_ratio() = gazebo::msgs::Convert(gazebo::math::Vector3(msg.tx, msg.ty, msg.tz));
    *result.mutable_torque_ratio() = gazebo::msgs::Convert(gazebo::math::Vector3(-msg.mx, -msg.my, -msg.mz));

    return result;
}

template<>
msgs::ipc::Message<MSG_NAVIG_TYPE> convert(const msgs::Navig& msg) {
    MSG_NAVIG_TYPE result;

    result.X_KNS = msg.position().x();
    result.Y_KNS = msg.position().y();
    result.Depth_NS = -msg.position().z();

    result.Roll_NS = msg.angle().x(); // крен
    result.Psi_NS = msg.angle().y();  // дифферент
    result.Fi_NS = msg.angle().z();   // курс

    return msgs::ipc::make_msg(result, MSG_NAVIG_FORMAT);
}

template<>
msgs::ipc::Message<MSG_COMPASS_TYPE> convert(const msgs::Compass& msg) {
    MSG_COMPASS_TYPE result;
    result.time = msg.time();

    result.state = 0;

    result.roll = msg.orientation().x();
    result.pitch = msg.orientation().y();
    result.heading = msg.orientation().z();

    result.roll_rate = msg.angular_vel().x();
    result.pitch_rate = msg.angular_vel().y();
    result.head_rate = msg.angular_vel().z();

    result.accX = msg.linear_accel().x();
    result.accY = msg.linear_accel().y();
    result.accZ = msg.linear_accel().z();

    return msgs::ipc::make_msg(result, MSG_COMPASS_FORMAT);
}

template<>
msgs::ipc::RawCamera convert(const msgs::Camera& msg) {
    MSG_VIDEO_FRAME m;

    cv::Mat frame = frame_from_msg(msg);

    m.camera_type = msg.camera_type() == msgs::Camera::FRONT ? CAMERA_FRONT : CAMERA_DOWN;
    m.w = msg.frame().width();
    m.h = msg.frame().height();
    m.channels = 3;
    m.size = m.w * m.h * m.channels;
    m.frame = new uchar[msg.frame().data().size()];
    memcpy(m.frame, frame.data, m.size);

    return msgs::ipc::make_msg(m, MSG_VIDEO_FRAME_FORMAT);
}

template<>
msgs::ipc::JpegCamera convert(const msgs::Camera& msg) {
    MSG_JPEG_VIDEO_FRAME m;

    cv::Mat frame = frame_from_msg(msg);

    std::vector<uchar> buff(frame.cols * frame.rows * 3);

    std::vector<int> param = {CV_IMWRITE_JPEG_QUALITY, 100};
    cv::imencode(".jpg", frame, buff, param);

    m.camera_type = msg.camera_type() == msgs::Camera::FRONT ? CAMERA_FRONT : CAMERA_DOWN;
    m.frame_type = ORIGINAL_FRAME;
    m.frame_size = buff.size();
    m.frame = new uchar[buff.size()];
    memcpy(m.frame, buff.data(), buff.size());

    return msgs::ipc::make_msg(m, MSG_JPEG_VIDEO_FRAME_FORMAT);
}

template<>
msgs::Camera convert(const MSG_SWITCH_CAMERA& msg) {
    msgs::Camera result;

    result.set_camera_type(msg.camera_type == CAMERA_DOWN ? msgs::Camera::DOWN : msgs::Camera::FRONT);

    return result;
}