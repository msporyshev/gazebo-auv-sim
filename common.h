#pragma once

#include <memory>

#define SUCCESS    "Success"

#ifdef USING_BOOST_LOG

#include <boost/log/trivial.hpp>

#define TRACE()    BOOST_LOG_TRIVIAL(trace)
#define DEBUG()    BOOST_LOG_TRIVIAL(debug)
#define INFO()     BOOST_LOG_TRIVIAL(info)
#define WARNING()  BOOST_LOG_TRIVIAL(warning)
#define ERROR()    BOOST_LOG_TRIVIAL(error)
#define FATAL()    BOOST_LOG_TRIVIAL(fatal)

#else

#include <iostream>

#define TRACE()    std::cout << std::endl
#define DEBUG()    TRACE()
#define INFO()     TRACE()
#define WARNING()  std::cerr << std::endl
#define ERROR()    WARNING()
#define FATAL()    WARNING()

#endif