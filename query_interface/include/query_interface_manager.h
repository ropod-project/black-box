#ifndef QUERY_INTERFACE_MANAGER_H
#define QUERY_INTERFACE_MANAGER_H

#include <iostream>
#include <memory>
#include <string>
#include <sstream>
#include <map>
#include <vector>
#include <json/json.h>

#include "utils/zyre/zyre_interface.h"
#include "utils/zyre/zyre_utils.h"
#include "config/config_enums.hpp"
#include "config/config_params.hpp"
#include "query_interface_base.h"
#include "query_interface_factory.h"

namespace data_retrieval
{
    /**
     * An interface for handling data query requests
     *
     * @author Alex Mitrevski, Santosh Thoduka
     * @contact aleksandar.mitrevski@h-brs.de, santosh.thoduka@h-brs.de
     */
    class QueryInterfaceManager : public utils::IZyreMsgReceive
    {
    public:
        /**
         * @param data_sources a list of data source names
         * @param config_params zyre configuration parameters
         * @param data_logger a DataLogger shared pointer
         */
        QueryInterfaceManager(const std::vector<std::string> data_sources,
                              const config::ZyreParams &config_params,
                              std::shared_ptr<loggers::DataLogger> data_logger);

        ~QueryInterfaceManager();

    private:
        virtual void initialise() { }

        virtual void cleanup() {}

        /**
         * Sends a response to a data query message;
         * ignores all other messages. Only responds to
         * messages of type NAME_QUERY, VARIABLE_QUERY, and DATA_QUERY
         *
         * @param msg a zyre message
         */
        virtual void react(zmsg_t *msg);

        /**
         * Converts msg_params.message to a json message
         *
         * @param msg_params message data
         */
        Json::Value parseMessage(const config::ZyreMsgParams &msg_params);

        /**
         * Converts msg_params.message to a json message
         *
         * @param msg_params message data
         */
        Json::Value getMessageHeader(std::string query_type);

        std::shared_ptr<loggers::DataLogger> data_logger_;
        std::map<std::string, std::shared_ptr<QueryInterfaceBase>> query_interfaces_;
        zactor_t *actor_;
    };
}

#endif
