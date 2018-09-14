#include "extern/zyre/exception.hpp"

namespace zyre
{
    error_t::error_t(const std::string& what)
    : std::runtime_error(what) {}
}
