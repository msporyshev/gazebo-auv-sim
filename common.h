#pragma once

#include <boost/log/trivial.hpp>

#define SUCCESS    "Success"

#define TRACE()    BOOST_LOG_TRIVIAL(trace)
#define DEBUG()    BOOST_LOG_TRIVIAL(debug)
#define INFO()     BOOST_LOG_TRIVIAL(info)
#define WARNING()  BOOST_LOG_TRIVIAL(warning)
#define ERROR()    BOOST_LOG_TRIVIAL(error)
#define FATAL()    BOOST_LOG_TRIVIAL(fatal)