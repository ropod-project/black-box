#ifndef CONFIG_PARAMS_H
#define CONFIG_PARAMS_H

#include <vector>
#include <string>
#include <map>
#include <zyre.h>

namespace config
{
    /**
     * Default black box configuration parameters
     *
     * @author Alex Mitrevski, Santosh Thoduka
     * @contact aleksandar.mitrevski@h-brs.de, santosh.thoduka@h-brs.de
     */
    struct DefaultParams
    {
        double max_frequency;
        unsigned int max_database_size;
        bool split_database;
    };

    /**
     * Black box configuration parameters for a specific ROS topic
     *
     * @author Alex Mitrevski, Santosh Thoduka
     * @contact aleksandar.mitrevski@h-brs.de, santosh.thoduka@h-brs.de
     */
    struct RosTopicParams
    {
        std::string name;
        std::string type;
        std::vector<std::string> variable_names;
        double max_frequency;
    };

    /**
     * ROS-specific black box configuration parameters
     *
     * @author Alex Mitrevski, Santosh Thoduka
     * @contact aleksandar.mitrevski@h-brs.de, santosh.thoduka@h-brs.de
     */
    struct RosParams
    {
        std::string ros_master_uri;
        std::vector<RosTopicParams> topics;
    };

    /**
     * Black box configuration parameters for a specific ZMQ topic
     *
     * @author Alex Mitrevski, Santosh Thoduka
     * @contact aleksandar.mitrevski@h-brs.de, santosh.thoduka@h-brs.de
     */
    struct ZmqTopicParams
    {
        std::string url;
        int port;
        std::string name;
        std::string type;
        std::vector<std::string> variable_names;
        double max_frequency;
    };

    /**
     * ZMQ-specific black box configuration parameters
     *
     * @author Alex Mitrevski, Santosh Thoduka
     * @contact aleksandar.mitrevski@h-brs.de, santosh.thoduka@h-brs.de
     */
    struct ZmqParams
    {
        std::vector<ZmqTopicParams> topics;
    };

    /**
     * Zyre-specific black box configuration parameters
     *
     * @author Alex Mitrevski, Santosh Thoduka
     * @contact aleksandar.mitrevski@h-brs.de, santosh.thoduka@h-brs.de
     */
    struct ZyreParams
    {
        std::string node_name;
        std::vector<std::string> groups;
        std::vector<std::string> message_types;
    };

    /**
     * Zyre message parameters
     *
     * @author Alex Mitrevski, Santosh Thoduka
     * @contact aleksandar.mitrevski@h-brs.de, santosh.thoduka@h-brs.de
     */
    class ZyreMsgParams
    {
    public:
        ZyreMsgParams(zmsg_t *msg)
        {
            zyre_msg = msg;
            event = zmsg_popstr(msg);
            peer = zmsg_popstr(msg);
            name = zmsg_popstr(msg);
            // if the event is WHISPER, no group is specified
            // so the fourth part of the zmsg_t is actually the message payload
            if (streq(event, "WHISPER"))
            {
                message = zmsg_popstr(msg);
            }
            else // for other events
            {
                group = zmsg_popstr(msg);
                message = zmsg_popstr(msg);
            }
        }

        ~ZyreMsgParams()
        {
            free(event);
            free(peer);
            free(name);
            free(group);
            free(message);
            zmsg_destroy(&zyre_msg);
        }

        zmsg_t *zyre_msg;
        char *event;
        char *peer;
        char *name;
        char *group;
        char *message;
    };

    /**
     * Parameters for a specific ethercat message block
     *
     * @author Alex Mitrevski, Santosh Thoduka
     * @contact aleksandar.mitrevski@h-brs.de, santosh.thoduka@h-brs.de
     */
    struct EthercatMessageElement
    {
        std::string name;
        std::string type;
    };

    /**
     * Ethercat-specific black box configuration parameters
     *
     * @author Alex Mitrevski, Santosh Thoduka
     * @contact aleksandar.mitrevski@h-brs.de, santosh.thoduka@h-brs.de
     */
    struct EthercatParams
    {
        std::string interface;
        uint32_t number_of_slaves;
        uint32_t input_message_size;
        uint32_t output_message_size;
        std::vector<EthercatMessageElement> output_message_elements;
        std::vector<EthercatMessageElement> input_message_elements;

        std::string input_data_name;
        std::string output_data_name;
        std::vector<std::string> input_variable_names;
        std::vector<std::string> output_variable_names;
    };

    /**
     * Collects all black box configuration parameters, namely the
     * default parameters and the data source-specific parameters
     *
     * @author Alex Mitrevski, Santosh Thoduka
     * @contact aleksandar.mitrevski@h-brs.de, santosh.thoduka@h-brs.de
     */
    struct ConfigParams
    {
        std::vector<std::string> interface_names;
        DefaultParams default_params;
        RosParams ros;
        ZmqParams zmq;
        std::vector<EthercatParams> ethercat;
        ZyreParams zyre;
    };
}

#endif /* CONFIG_PARAMS_H */
