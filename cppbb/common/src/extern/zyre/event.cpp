#include "extern/zyre/event.hpp"

namespace zyre
{
    event_t::event_t(zyre_event_t* self) : m_self(self) {}

    event_t::~event_t()
    {
        if (m_self)
            zyre_event_destroy(&m_self);
    }

    event_t::event_t(event_t&& other)
    {
        m_self = other.m_self;
        other.m_self = nullptr;
    }

    event_t& event_t::operator=(event_t&& other)
    {
        if (&other != this)
        {
            m_self = other.m_self;
            other.m_self = nullptr;
        }
        return *this;
    }

    void event_t::print() const
    {
        zyre_event_print(m_self);
    }

    std::string event_t::type() const
    {
        const char *val = zyre_event_type(m_self);
        if(val == NULL) {
            return "";
        }

        return val;
    }

    std::string event_t::sender() const
    {
        const char *val = zyre_event_peer_uuid(m_self);
        if(val == NULL) {
            return "";
        }

        return val;
    }

    std::string event_t::name() const
    {
        const char *val = zyre_event_peer_name(m_self);
        if(val == NULL) {
            return "";
        }

        return val;
    }

    std::string event_t::address() const
    {
        const char *val = zyre_event_peer_addr(m_self);
        if(val == NULL) {
            return "";
        }

        return val;
    }

    std::string event_t::header_value(const std::string& key) const
    {
        const char *val = zyre_event_header(m_self, key.c_str());
        if(val == NULL) {
            return "";
        }

        return val;
    }

    std::string event_t::group() const
    {
        const char *val = zyre_event_group(m_self);
        if(val == NULL) {
            return "";
        }

        return val;
    }

    zmsg_t* event_t::message() const
    {
        return zyre_event_msg(m_self);
    }
}
