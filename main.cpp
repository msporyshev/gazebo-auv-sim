#include <gazebo/transport/transport.hh>
#include <gazebo/gazebo.hh>

#include <iostream>
#include <string>
#include <memory>
#include <cerrno>

#include <ipc.h>

#include <boost/scope_exit.hpp>

#include "exception.h"
#include "common.h"
#include "transport_pipe.h"

namespace gztransport = gazebo::transport;

template<typename PipeTag>
using  TransportPipePtr = std::shared_ptr<TransportPipe<PipeTag> >;

struct AdapterParams {
    std::string hostname = "localhost";
    std::string taskname = "adapter";
} adapter_params;

TransportPipePtr<RegulPipeTag> regul_pipe;
gztransport::NodePtr node;

void ipc_init() {
    INFO() << "Connecting to ipc central";
    if(IPC_connectModule(adapter_params.taskname.c_str(), adapter_params.hostname.c_str()) != IPC_OK) {
        THROW(Exception(errno, "Unable to connect to central"));
    }
}

void ipc_shutdown() {
    IPC_disconnect();
}

void gazebo_init(int argc, char** argv) {
    INFO() << "Starting gazebo client";
    if (!gazebo::setupClient(argc, argv)) {
        THROW(Exception("Unable to setup gazebo client"));
    }

    INFO() << "Initializing gazebo transport node";
    node = gztransport::NodePtr(new gztransport::Node());
    node->Init();

    gztransport::run();

}

void init(int argc, char** argv) {
    gazebo_init(argc, argv);
    ipc_init();

    regul_pipe = TransportPipePtr<RegulPipeTag>(new TransportPipe<RegulPipeTag>(node));
}

void main_loop() {
    while (true) {
        IPC_listenClear(0);
        gazebo::common::Time::MSleep(10);
    }
}

void gazebo_shutdown() {
    gztransport::fini();

    gazebo::shutdown();
}

int main(int argc, char** argv) {
    try {
        BOOST_SCOPE_EXIT_ALL() {
            gazebo_shutdown();
            ipc_shutdown();
        };

        init(argc, argv);

        main_loop();
    } catch (Exception& e) {
        FATAL() << e;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}