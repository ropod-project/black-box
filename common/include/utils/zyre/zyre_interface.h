#ifndef ZYRE_INTERFACE_H
#define ZYRE_INTERFACE_H

#include <iostream>
#include <json/json.h>
#include "config/config_params.hpp"
#include "extern/zyre/node.hpp"

namespace utils
{
    class IZyreMsgReceive
    {
    public:
        virtual ~IZyreMsgReceive() { }

        virtual void initialise() = 0;
        virtual void react(zmsg_t *msg) = 0;
        virtual void cleanup() = 0;

        config::ZyreParams zyre_config_params_;
        std::string node_name_;
        zyre::node_t *node_;

        Json::StreamWriterBuilder json_stream_builder_;
    };
}

#endif
