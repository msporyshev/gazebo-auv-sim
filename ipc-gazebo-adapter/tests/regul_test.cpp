#include <iostream>
#include <fstream>
#include <map>
#include <string>
#include <vector>
#include <mutex>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/gazebo.hh>

#include <regul.pb.h>
#include <navig.pb.h>

#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>

typedef boost::tokenizer<boost::char_separator<char> > Tokenizer;

std::map<std::string, std::vector<std::string> > regul_data;

std::vector<msgs::Regul> regul_messages;
std::vector<msgs::Navig> navig_messages;

std::mutex navig_mutex;

gazebo::transport::NodePtr node;
gazebo::transport::PublisherPtr publisher;
gazebo::transport::SubscriberPtr subscriber;

void parse_regul_data(std::string filename) {
    std::ifstream input("regul.log");


    boost::char_separator<char> sep(" ");


    std::vector<std::string> headers;
    std::string headerstr;
    getline(input, headerstr);

    Tokenizer tokens(headerstr, sep);

    for (auto it = tokens.begin(); it != tokens.end(); it++) {
        headers.push_back(*it);
    }

    std::string line;
    while(getline(input, line)) {
        tokens.assign(line);
        int i = 0;
        for (auto tok : tokens) {
            regul_data[headers[i++]].push_back(tok);
        }
    }

    for (int i = 0; i < regul_data["tx"].size(); i++) {
        msgs::Regul msg;

        double tx = boost::lexical_cast<double>(regul_data["tx"][i]);
        double ty = boost::lexical_cast<double>(regul_data["ty"][i]);
        double tz = boost::lexical_cast<double>(regul_data["tz"][i]);
        double mx = boost::lexical_cast<double>(regul_data["mx"][i]);
        double my = boost::lexical_cast<double>(regul_data["my"][i]);
        double mz = boost::lexical_cast<double>(regul_data["mz"][i]);
        Set(msg.mutable_force_ratio(), gazebo::math::Vector3(tx, ty, tz));
        Set(msg.mutable_torque_ratio(), gazebo::math::Vector3(mx, my, mz));

        regul_messages.push_back(msg);
    }

}

void RecieveNavig(const boost::shared_ptr<const msgs::Navig>& msg) {
    std::lock_guard<std::mutex> lock(navig_mutex);
    navig_messages.push_back(*msg);
}

void gazebo_init() {
    gazebo::setupClient(0, nullptr);

    node = gazebo::transport::NodePtr(new gazebo::transport::Node());
    node->Init("robosub_auv");

    publisher = node->Advertise<msgs::Regul>("~/regul");
    subscriber = node->Subscribe("~/navig", &RecieveNavig);
}

int main(int argc, char** argv) {
    parse_regul_data("regul.log");

    gazebo_init();

    for (auto msg : regul_messages) {
        gazebo::common::Time::MSleep(300);
        publisher->Publish(msg);
    }
    gazebo::shutdown();
    std::cout << "shut down" << std::endl;
}