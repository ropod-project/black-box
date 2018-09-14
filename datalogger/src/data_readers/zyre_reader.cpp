#include "datalogger/data_readers/zyre_reader.hpp"
#include <iostream>
#include <json/json.h>
#include <sstream>
#include <chrono>
#include <algorithm>

namespace readers
{
    /**
     * @param config_params zyre-specific configuration parameterss
     * @param data_logger a data logger instance for logging the received data
     */
    ZyreReader::ZyreReader(const config::ZyreParams &config_params, std::shared_ptr<loggers::DataLogger> data_logger)
        : DataReader(data_logger)
    {
        node_name_ = config_params.node_name;
        zyre_config_params_ = config_params;

        node_ = new zyre::node_t(node_name_);
        node_->start();
        zclock_sleep(100);

        for (int i=0; i<zyre_config_params_.groups.size(); i++)
        {
            node_->join(zyre_config_params_.groups[i]);
        }
        zclock_sleep(100);

        actor_ = zactor_new(utils::Zyre::receiveLoop, this);
    }

    ZyreReader::~ZyreReader()
    {
        for (int i=0; i<zyre_config_params_.groups.size(); i++)
        {
            node_->leave(zyre_config_params_.groups[i]);
        }
        node_->stop();
        zactor_destroy(&actor_);
        delete node_;
    }

    void ZyreReader::react(zmsg_t *msg)
    {
        config::ZyreMsgParams msg_params(msg);
        if (streq(msg_params.event, "SHOUT"))
        {
            std::stringstream msg_stream;
            msg_stream << msg_params.message;

            Json::Value root;
            Json::CharReaderBuilder reader_builder;
            std::string errors;
            bool ok = Json::parseFromStream(reader_builder, msg_stream, &root, &errors);
            std::string message_type = root["header"]["type"].asString();

            // only log messages that are specifying in config file
            if (std::find(zyre_config_params_.message_types.begin(),
                          zyre_config_params_.message_types.end(), message_type)
                          != zyre_config_params_.message_types.end())
            {
                auto now = std::chrono::system_clock::now();
                double timestamp =  std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() / 1000.0;

                std::string group_str(msg_params.group);
                data_tunnel_interface_.tunnelData(DataReader::formatVariableName(config::DataSourceNames::ZYRE, group_str + "_" + message_type),
                                                  timestamp,
                                                  msg_stream.str());
            }
        }
    }
}
