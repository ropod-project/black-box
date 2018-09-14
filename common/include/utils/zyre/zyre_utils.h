#ifndef ZYRE_UTILS_H
#define ZYRE_UTILS_H

#include <iostream>
#include <string>
#include <vector>
#include <json/json.h>

#include "extern/zyre/node.hpp"
#include "extern/zyre/event.hpp"
#include "extern/zyre/exception.hpp"
#include "utils/zyre/zyre_interface.h"

namespace utils
{
    class Zyre
    {
    public:
        /**
         * Waits for query messages and passes them on to the "respond" method
         *
         * @param pipe a socket pointer
         * @param args pointer to the creating object
         */
        static void receiveLoop(zsock_t *pipe, void *args);

        /**
         * Shouts a message to all the groups of which caller->node_ is part
         *
         * @param json_msg the json message that should be shouted
         */
        static void shoutMessage(const Json::Value &json_msg, const IZyreMsgReceive *caller);

        /**
         * Whispers a message to a peer
         *
         * @param json_msg the json message that should be whispered
         */
        static void whisperMessage(const Json::Value &json_msg, const IZyreMsgReceive *caller, const std::string &peer);

        /**
         * Converts an std::string to a zmsg_t pointer
         *
         * @param msg the string to be converted
         */
        static zmsg_t* stringToZmsg(std::string msg);
    };
}

#endif
