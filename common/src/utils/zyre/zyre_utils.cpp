#include "utils/zyre/zyre_utils.h"

namespace utils
{
    void Zyre::receiveLoop(zsock_t *pipe, void *args)
    {
        utils::IZyreMsgReceive *caller = static_cast<IZyreMsgReceive*>(args);
        zsock_signal(pipe, 0);
        bool terminated = false;

        caller->initialise();
        zpoller_t *poller = zpoller_new (pipe, caller->node_->socket(), NULL);
        while (!terminated)
        {
            void *which = zpoller_wait (poller, -1); // no timeout
            if (which == pipe) // message sent to the actor
            {
                zmsg_t *msg = zmsg_recv (which);
                if (!msg)
                    break;              //  Interrupted

                char *command = zmsg_popstr (msg);
                if (streq (command, "$TERM"))
                {
                    terminated = true;
                }
                else
                {
                    std::cerr << "invalid message to actor" << std::endl;
                    assert (false);
                }
                free (command);
                zmsg_destroy (&msg);
            }
            else if (which == caller->node_->socket()) // message sent to the node
            {
                zmsg_t *msg = zmsg_recv (which);
                caller->react(msg);
            }
        }
        caller->cleanup();
        zpoller_destroy (&poller);
    }

    /**
     * Shouts a message to all the groups of which caller->node_ is part
     *
     * @param json_msg the json message that should be shouted
     */
    void Zyre::shoutMessage(const Json::Value &json_msg, const IZyreMsgReceive *caller)
    {
        std::string msg = Json::writeString(caller->json_stream_builder_, json_msg);
        zmsg_t* message = Zyre::stringToZmsg(msg);
        for (std::string group : caller->zyre_config_params_.groups)
        {
            caller->node_->shout(group, message);
        }
    }

    /**
     * Whispers a message to a peer
     *
     * @param json_msg the json message that should be whispered
     */
    void Zyre::whisperMessage(const Json::Value &json_msg, const IZyreMsgReceive *caller, const std::string &peer)
    {
        std::string msg = Json::writeString(caller->json_stream_builder_, json_msg);
        zmsg_t* message = Zyre::stringToZmsg(msg);
        caller->node_->whisper(peer, message);
    }

    /**
     * Converts an std::string to a zmsg_t pointer
     *
     * @param msg the string to be converted
     */
    zmsg_t* Zyre::stringToZmsg(std::string msg)
    {
        zmsg_t* message = zmsg_new();
        zframe_t *frame = zframe_new(msg.c_str(), msg.size());
        zmsg_prepend(message, &frame);
        return message;
    }
}
