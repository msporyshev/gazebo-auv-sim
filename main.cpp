#include <gazebo/transport/transport.hh>
#include <gazebo/gazebo.hh>

#include <iostream>
#include <string>
#include <memory>
#include <cerrno>

#include <ipc.h>

#include <boost/scope_exit.hpp>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

#include "exception.h"
#include "common.h"
#include "policy.h"
#include "transport_pipe.h"


namespace po = boost::program_options;
namespace gztransport = gazebo::transport;
namespace logging = boost::log::trivial;

namespace {

template<typename PipePolicy>
using  TransportPipePtr = std::shared_ptr<TransportPipe<PipePolicy> >;

struct AdapterParams {
    std::string hostname = "localhost";
    std::string taskname = "adapter";
    std::string topic_namespace = "robosub_auv";
    logging::severity_level log_level = logging::info;
} adapter_params;

std::list<std::shared_ptr<AbstractPipe> > pipes;
gztransport::NodePtr node;

template<typename T>
void load_param(const po::variables_map& vm, std::string name, T& param) {
    if (!vm.count(name)) {
        return;
    }

    param = vm[name].as<T>();
}

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

void log_init() {
    boost::log::core::get()->set_filter
    (
        logging::severity >= adapter_params.log_level
    );
}

void program_options_init(int argc, char** argv) {
    po::options_description desc("Usage");
    desc.add_options()
        ("help,h", "Produce help message")
        ("ipc-host,i", po::value<std::string>(), "Set ipc central ip address, default=localhost")
        ("namespace,n", po::value<std::string>(), "Set gazebo topic namespace, default=robosub_auv")
        ("verbose,v", po::value<logging::severity_level>(), "Be verbose <debug|info|warning|error|fatal>, default=info")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << "\n";
        exit(EXIT_SUCCESS);
    }

    load_param(vm, "ipc-host", adapter_params.hostname);
    load_param(vm, "namespace", adapter_params.topic_namespace);
    load_param(vm, "verbose", adapter_params.log_level);
}

void gazebo_init(int argc, char** argv) {
    INFO() << "Starting gazebo client";
    if (!gazebo::load(argc, argv)) {
        THROW(Exception("Unable to setup gazebo client"));
    }
    gazebo::run();

    INFO() << "Initializing gazebo transport node";

    gztransport::init();
    gztransport::run();

    node = gztransport::NodePtr(new gztransport::Node());
    node->Init(adapter_params.topic_namespace);
}

void init(int argc, char** argv) {
    program_options_init(argc, argv);

    log_init();

    gazebo_init(argc, argv);
    ipc_init();

    pipes.emplace_back(new TransportPipe<RegulPolicy>(node));
    pipes.emplace_back(new TransportPipe<NavigPolicy>(node));
    pipes.emplace_back(new TransportPipe<CameraPolicy>(node));
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
    gazebo::fini();
}

} // namespace

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