#ifndef ZYRE_EVENT_HPP
#define ZYRE_EVENT_HPP

#include <zyre.h>
#include <string>
#include <vector>

#include "extern/zyre/exception.hpp"

namespace zyre
{
    class event_t
    {
    public:
        event_t(zyre_event_t* self);
        ~event_t();
        event_t(const event_t& other) = delete;
        event_t operator=(const event_t& other) = delete;

        event_t(event_t&& other);
        event_t& operator=(event_t&& other);

        void print() const;
        std::string type() const;
        std::string sender() const;
        std::string name() const;
        std::string address() const;
        std::string header_value(const std::string& key) const;
        std::string group() const;
        zmsg_t* message() const;
    private:
        zyre_event_t* m_self;
    };
}

#endif
