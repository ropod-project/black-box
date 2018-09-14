#ifndef ZYRE_NODE_HPP
#define ZYRE_NODE_HPP

#include <zyre.h>
#include <string>
#include <vector>

#include "extern/zyre/exception.hpp"
#include "extern/zyre/event.hpp"

namespace zyre
{
    class node_t
    {
    public:
        node_t(const std::string& name = "");
        ~node_t();
        node_t(const node_t& other) = delete;
        node_t operator=(const node_t& other) = delete;

        node_t(node_t&& other);
        node_t& operator=(node_t&& other);

        void print() const;
        std::string uuid() const;
        std::string name() const;
        void set_header(const std::string key, const std::string& value) const;
        void set_verbose() const;
        void set_port(int value) const;
        void set_interval(size_t value) const;
        void set_interface(const std::string& value) const;
        int start() const;
        void stop() const;
        int join(const std::string& group) const;
        int leave(const std::string& group) const;
        int whisper(const std::string& peer, zmsg_t* msg) const;
        int shout(const std::string& group, zmsg_t* msg) const;
        zmsg_t* recv() const;
        event_t event() const;
        std::vector<std::string> peers() const;
        std::vector<std::string> own_groups() const;
        std::vector<std::string> peer_groups() const;
        std::string peer_address(const std::string& peer) const;
        std::string peer_header_value(const std::string& peer, const std::string& name);
        zsock_t* socket() const;
        static int version();
    private:
        std::vector<std::string> to_vector(zlist_t* list) const;
        zyre_t* m_self;
    };
}

#endif
