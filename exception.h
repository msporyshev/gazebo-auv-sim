#pragma once

#include <boost/exception/all.hpp>

#include <exception>
#include <string>
#include <ostream>
#include <cerrno>
#include <cstring>


#define THROW(e) \
    BOOST_THROW_EXCEPTION(e)

typedef boost::error_info<struct tag, std::string> error_string_info;

class Exception: public boost::exception, public std::exception {
public:
    explicit Exception(std::string msg = "") {
        *this << error_string_info(msg);
    }

    explicit Exception(int errcode, std::string msg = "") {
        *this << error_string_info(msg) << boost::errinfo_errno(errcode);
    }

    virtual std::string info() const {
        return diagnostic_information(*this);
    }

    friend std::ostream&operator<< (std::ostream &output, const Exception &e) {
        return output << e.info();
    }

};