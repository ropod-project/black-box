#include <sys/time.h>
#include "datalogger/data_readers/json_zmq_reader.hpp"
#include <iostream>

namespace readers
{
    /**
     * @param config_params ZMQ-specific configuration parameterss
     * @param data_logger a data logger instance for logging the received data
     */
    JsonZMQReader::JsonZMQReader(const config::ZmqTopicParams &config_params, std::shared_ptr<loggers::DataLogger> data_logger)
        : ZMQReader(config_params, data_logger), reader_(rbuilder_.newCharReader())
    {
    }

    JsonZMQReader::~JsonZMQReader()
    {
    }

    /**
     * Parses a received ZMQ message
     *
     * @param zmq_message received message
     */
    void JsonZMQReader::parseMessage(zmq::message_t &zmq_message)
    {
        Json::Value root;
        std::string errs;

        reader_->parse((char*)zmq_message.data(), (char*)zmq_message.data() + zmq_message.size(), &root, &errs);

        std::vector<std::string> values;
        values.push_back(root["source"].asString());
        values.push_back(root["user_role"].asString());
        values.push_back(root["message"].asString());

        double timestamp = root["stamp"].asDouble();

        data_tunnel_interface_.tunnelData(DataReader::formatVariableName(config::DataSourceNames::ZMQ, config_params_.name),
                                          timestamp,
                                          values);
    }
}
