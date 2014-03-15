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

namespace po = boost::program_options;
namespace gztransport = gazebo::transport;

template<typename PipeTag>
using  TransportPipePtr = std::shared_ptr<TransportPipe<PipeTag> >;

struct AdapterParams {
    std::string hostname = "localhost";
    std::string taskname = "adapter";
    std::string topic_namespace = "robosub_auv";
    std::string log_level  = "INFO";
} adapter_params;

TransportPipePtr<RegulPipeTag> regul_pipe;
gztransport::NodePtr node;

void ipc_init() {
    INFO() << "Connecting to ipc central";
    if(IPC_connectModule(adapter_params.taskname.c_str(), adapter_params.hostname.c_str()) != IPC_OK) {
        THROW(Exception(errno, "Unable to connect to central"));
    }
    INFO() << SUCCESS;
}

void ipc_shutdown() {
    IPC_disconnect();
}

void program_options_init(int argc, char** argv) {
    po::options_description desc("Usage:");
    desc.add_options()
        ("help,h", "Produce help message")
        ("ipc-host,i", po::value<std::string>(), "Set ipc central ip address, default=localhost")
        ("namespace,n", po::value<std::string>(), "Set gazebo topic namespace, default=robosub_auv")
        ("verbose,v", po::value<std::string>(), "Be verbose <INFO|WARNING|ERROR|FATAL|DEBUG>, default=INFO")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << "\n";
        exit(EXIT_SUCCESS);
    }

    if (vm.count("ipc-host")) {
        adapter_params.hostname = vm["ipc-host"].as<std::string>();
    }

    if (vm.count("namespace")) {
        adapter_params.topic_namespace = vm["namespace"].as<std::string>();
    }

    if (vm.count("verbose")) {
        adapter_params.log_level = vm["verbose"].as<std::string>();
    }

}

void gazebo_init(int argc, char** argv) {
    INFO() << "Starting gazebo client";
    if (!gazebo::setupClient(argc, argv)) {
        THROW(Exception("Unable to setup gazebo client"));
    }

    INFO() << "Initializing gazebo transport node";

    gztransport::init();
    gztransport::run();

    node = gztransport::NodePtr(new gztransport::Node());
    node->Init(adapter_params.topic_namespace);
}

void init(int argc, char** argv) {
    program_options_init(argc, argv);

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
    node->Fini();
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
    } catch (boost::program_options::error& e) {
        FATAL() << "Uncorrect option parameters";
        FATAL() << e.what();
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}