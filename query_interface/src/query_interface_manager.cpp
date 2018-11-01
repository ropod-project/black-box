#include "query_interface_manager.h"

namespace data_retrieval
{
    /**
     * @param data_sources a list of data source names
     * @param config_params zyre configuration parameters
     * @param data_logger a DataLogger shared pointer
     */
    QueryInterfaceManager::QueryInterfaceManager(const std::vector<std::string> data_sources,
                                                 const config::ZyreParams &config_params,
                                                 std::shared_ptr<loggers::DataLogger> data_logger)
    {
        node_name_ = config_params.node_name + "_query_interface";
        zyre_config_params_ = config_params;
        data_logger_ = data_logger;

        node_ = new zyre::node_t(node_name_);
        node_->start();
        zclock_sleep(100);

        for (int i = 0; i < config_params.groups.size(); i++)
        {
            node_->join(config_params.groups[i]);
        }
        zclock_sleep(100);

        actor_ = zactor_new(utils::Zyre::receiveLoop, this);

        for (std::string data_source : data_sources)
        {
            std::shared_ptr<QueryInterfaceBase> query_interface = QueryInterfaceFactory::getQueryInterface(data_source);
            query_interfaces_[data_source] = query_interface;
        }
    }

    QueryInterfaceManager::~QueryInterfaceManager()
    {
        for (int i=0; i<zyre_config_params_.groups.size(); i++)
        {
            node_->leave(zyre_config_params_.groups[i]);
        }
        node_->stop();
        zactor_destroy(&actor_);
        delete node_;
    }

    /**
     * Sends a response to a data query message;
     * ignores all other messages. Only responds to
     * messages of type VARIABLE_QUERY and DATA_QUERY
     *
     * @param msg a zyre message
     */
    void QueryInterfaceManager::react(zmsg_t *msg)
    {
        config::ZyreMsgParams msg_params(msg);
        Json::Value json_msg = parseMessage(msg_params);

        if (json_msg == Json::nullValue)
            return;

        std::string message_type = json_msg["header"]["type"].asString();
        std::string sender_id = json_msg["payload"]["senderId"].asString();
        if (message_type == "NAME_QUERY")
        {
            Json::Value response_msg;
            response_msg["header"] = getMessageHeader(message_type);
            response_msg["payload"]["metamodel"] = "ropod-query-schema.json";
            response_msg["payload"]["receiverId"] = sender_id;
            utils::Zyre::shoutMessage(response_msg, this);
        }
        else if (message_type == "DATA_QUERY")
        {
            //we ignore requests that are not for this black box
            std::string query_interface_id = json_msg["payload"]["blackBoxId"].asString() + "_query_interface";
            if (query_interface_id != node_name_)
            {
                return;
            }

            std::map<std::string, std::map<std::string, std::vector<std::string>>> interface_variables;
            for (auto variable : json_msg["payload"]["variables"])
            {
                // we look for the location of the first underscore, which
                // is the delimiter used for separating the interface name
                // from the name of the variable
                std::string var = variable.asString();
                unsigned int underscore_idx = var.find('_');
                unsigned int slash_idx = var.find('/');
                std::string interface_name = var.substr(0, underscore_idx);
                std::string group_name = var.substr(0, slash_idx);
                var = var.substr(slash_idx+1, var.size()-slash_idx);

                // we add the variable to the list of variables for the given interface
                if (interface_variables.count(interface_name) == 0)
                {
                    interface_variables[interface_name] = std::map<std::string, std::vector<std::string>>();
                }

                if (interface_variables[interface_name].count(group_name) == 0)
                {
                    interface_variables[interface_name][group_name] = std::vector<std::string>();
                }
                interface_variables[interface_name][group_name].push_back(var);
            }

            std::string start_time = json_msg["payload"]["startTime"].asString();
            std::string end_time = json_msg["payload"]["endTime"].asString();

            Json::Value response_msg;
            response_msg["header"] = getMessageHeader(message_type);
            response_msg["payload"]["metamodel"] = "ropod-query-schema.json";
            response_msg["payload"]["receiverId"] = sender_id;

            // the data list will be a list of dictionaries, where each key
            // corresponds to a variable name and the value corresponds to
            // a list of values for that variable in the desired time frame
            Json::Value &data_list = response_msg["payload"]["dataList"];

            for (auto interface : query_interfaces_)
            {
                // we query the data from every interface for which there are
                // query variables; we then add the data corresponding to
                // each variable to the data list
                if (interface_variables.count(interface.first) > 0)
                {
                    std::map<std::string, std::vector<std::string>> data = interface.second->getData(data_logger_,
                                                                                                     interface_variables[interface.first],
                                                                                                     start_time,
                                                                                                     end_time);
                    for (auto variable_data : data)
                    {
                        Json::Value variable_data_list;
                        for (std::string data_item : variable_data.second)
                        {
                            variable_data_list.append(data_item);
                        }

                        std::string var_name = variable_data.first;
                        Json::Value var_data;
                        var_data[var_name] = variable_data_list;
                        data_list.append(var_data);
                    }
                }
            }

            utils::Zyre::whisperMessage(response_msg, this, std::string(msg_params.peer));
        }
        else if (message_type == "VARIABLE_QUERY")
        {
            //we ignore requests that are not for this black box
            std::string query_interface_id = json_msg["payload"]["blackBoxId"].asString() + "_query_interface";
            if (query_interface_id != node_name_)
            {
                return;
            }

            std::map<std::string, std::vector<std::string>> source_variable_names;
            for (auto interface : query_interfaces_)
            {
                std::vector<std::string> variables = interface.second->getVariables(data_logger_);
                source_variable_names[interface.first] = variables;
            }

            Json::Value response_msg;
            response_msg["header"] = getMessageHeader(message_type);
            response_msg["payload"]["metamodel"] = "ropod-query-schema.json";
            response_msg["payload"]["receiverId"] = sender_id;

            Json::Value &source_variable_list = response_msg["payload"]["variableList"];
            for (auto source_var_names : source_variable_names)
            {
                Json::Value variable_list;
                for (std::string var : source_var_names.second)
                {
                    variable_list.append(var);
                }

                std::string source_name = source_var_names.first;
                Json::Value source_vars;
                source_vars[source_name] = variable_list;
                source_variable_list.append(source_vars);
            }

            utils::Zyre::whisperMessage(response_msg, this, std::string(msg_params.peer));
        }
    }

    /**
     * Converts msg_params.message to a json message
     *
     * @param msg_params message data
     */
    Json::Value QueryInterfaceManager::parseMessage(const config::ZyreMsgParams &msg_params)
    {
        if (streq(msg_params.event, "SHOUT"))
        {
            std::stringstream msg_stream;
            msg_stream << msg_params.message;

            Json::Value root;
            Json::CharReaderBuilder reader_builder;
            std::string errors;
            bool ok = Json::parseFromStream(reader_builder, msg_stream, &root, &errors);

            return root;
        }

        return Json::nullValue;
    }

    /**
     * Converts msg_params.message to a json message
     *
     * @param msg_params message data
     */
    Json::Value QueryInterfaceManager::getMessageHeader(std::string query_type)
    {
        Json::Value root;
        root["type"] = query_type;
        root["metamodel"] = "ropod-msg-schema.json";

        zuuid_t *uuid = zuuid_new();
        const char *uuid_str = zuuid_str_canonical(uuid);
        root["msg_id"] = uuid_str;
        zuuid_destroy(&uuid);

        char * timestr = zclock_timestr();
        root["timestamp"] = timestr;
        zstr_free(&timestr);

        return root;
    }
}
