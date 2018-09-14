#include "extern/zyre/node.hpp"

namespace zyre
{
    node_t::node_t(const std::string& name)
    {
        if (name == "")
            m_self = zyre_new(NULL);
        else
            m_self = zyre_new(name.c_str());
    }

    node_t::~node_t()
    {
        if (m_self)
            zyre_destroy(&m_self);
    }

    node_t::node_t(node_t&& other)
    {
        m_self = other.m_self;
        other.m_self = nullptr;
    }

    node_t& node_t::operator=(node_t&& other)
    {
        if (&other != this)
        {
            m_self = other.m_self;
            other.m_self = nullptr;
        }
        return *this;
    }

    void node_t::print() const
    {
        zyre_print(m_self);
    }

    std::string node_t::uuid() const
    {
        return zyre_uuid(m_self);
    }

    std::string node_t::name() const
    {
        return zyre_name(m_self);
    }

    void node_t::set_header(const std::string key, const std::string& value) const
    {
        zyre_set_header(m_self, key.c_str(), "%s", value.c_str());
    }

    void node_t::set_verbose() const
    {
        zyre_set_verbose(m_self);
    }

    void node_t::set_port(int value) const
    {
        zyre_set_port(m_self, value);
    }

    void node_t::set_interval(size_t value) const
    {
        zyre_set_interval(m_self, value);
    }

    void node_t::set_interface(const std::string& value) const
    {
        zyre_set_interface(m_self, value.c_str());
    }

    int node_t::start() const
    {
        int rc = zyre_start(m_self);
        if (rc == -1)
            throw error_t("Failed to start Zyre node");
        return rc;
    }

    void node_t::stop() const
    {
        zyre_stop(m_self);
    }

    int node_t::join(const std::string& group) const
    {
        return zyre_join(m_self, group.c_str());
    }

    int node_t::leave(const std::string& group) const
    {
        return zyre_leave(m_self, group.c_str());
    }

    int node_t::whisper(const std::string& peer, zmsg_t* msg) const
    {
        return zyre_whisper(m_self, peer.c_str(), &msg);
    }

    int node_t::shout(const std::string& group, zmsg_t* msg) const
    {
        return zyre_shout(m_self, group.c_str(), &msg);
    }

    zmsg_t* node_t::recv() const
    {
        return zyre_recv(m_self);
    }

    event_t node_t::event() const
    {
        return event_t(zyre_event_new(m_self));
    }

    std::vector<std::string> node_t::peers() const
    {
        zlist_t* peers = zyre_peers(m_self);
        std::vector<std::string> ret;
        if(peers == NULL) {
            return ret;
        }
        ret = to_vector(peers);
        zlist_destroy(&peers);
        return ret;
    }

    std::vector<std::string> node_t::own_groups() const
    {
        zlist_t* ownGroups = zyre_own_groups(m_self);
        std::vector<std::string> ret;
        if(ownGroups == NULL) {
            return ret;
        }
        ret = to_vector(ownGroups);
        zlist_destroy(&ownGroups);
        return ret;
    }

    std::vector<std::string> node_t::peer_groups() const
    {
        zlist_t* peerGroups = zyre_peer_groups(m_self);
        std::vector<std::string> ret;
        if(peerGroups == NULL) {
            return ret;
        }
        ret = to_vector(peerGroups);
        zlist_destroy(&peerGroups);
        return ret;
    }

    std::string node_t::peer_address(const std::string& peer) const
    {
        char* val = zyre_peer_address(m_self, peer.c_str());
        std::string ret(val);
        if (val != NULL)
            delete val;
        return ret;
    }

    std::string node_t::peer_header_value(const std::string& peer, const std::string& name)
    {
        char* val = zyre_peer_header_value(m_self, peer.c_str(), name.c_str());
        std::string ret(val);
        if (val != NULL)
            delete val;
        return ret;
    }

    zsock_t* node_t::socket() const
    {
        return zyre_socket(m_self);
    }

    int node_t::version()
    {
        return zyre_version();
    }

    std::vector<std::string> node_t::to_vector(zlist_t* list) const
    {
        std::vector<std::string> ret;
        void* cursor = zlist_first(list);
        while (cursor != NULL)
        {
            ret.emplace_back(static_cast<char*>(cursor));
            cursor = zlist_next(list);
        }
        return ret;
    }
}
