#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>

#include <boost/shared_ptr.hpp>

#include <iostream>

#include <ipc.h>

#include "exception.h"

namespace gztransport = gazebo::transport;
namespace gzmsgs = gazebo::msgs;

typedef boost::shared_ptr<const gzmsgs::Image> ImageMsgPtr;

void cb(const ImageMsgPtr& _msg)
{
  // Dump the message contents to stdout.
    std::cout << _msg->DebugString();
}

void gazebo_init(int argc, char** argv) {
    if (!gazebo::setupClient(argc, argv)) {
        THROW(Exception("Unable to setup gazebo client"));
    }

    gztransport::Node node;
    node.Init();

    gztransport::SubscriberPtr sub = node.Subscribe("~/camera/front", cb);
    gztransport::run();

}

void gazebo_loop() {
    while (true) {
        gazebo::common::Time::MSleep(10);
    }
}

void gazebo_shutdown() {
    gztransport::fini();

    gazebo::shutdown();
}

int main(int argc, char** argv) {

    try {
        gazebo_init(argc, argv);

        gazebo_loop();

        gazebo_shutdown();
    } catch (Exception& e) {
        std::cerr << e << std::endl;
        return 1;
    }
    return 0;
}