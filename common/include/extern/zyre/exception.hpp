#ifndef ZYRE_EXCEPTION_HPP
#define ZYRE_EXCEPTION_HPP

#include <stdexcept>
#include <string>
#include <exception>

using std::runtime_error;

namespace zyre
{
    class error_t : public std::runtime_error
    {
    public:
        error_t(const std::string& what);
    };
}

#endif
