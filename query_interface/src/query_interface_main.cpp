#include <thread>
#include <chrono>

#include "config/config_enums.hpp"
#include "config/config_params.hpp"
#include "config/config_file_reader.hpp"
#include "datalogger/data_loggers/mongodb_logger.hpp"
#include "query_interface_manager.h"

bool terminate = false;

void checkTermination(int signal)
{
    terminate = true;
}

int main(int argc, char *argv[])
{
    std::vector<std::string> data_source_names = { config::DataSourceNames::ROS,
                                                   config::DataSourceNames::ZMQ,
                                                   config::DataSourceNames::ETHERCAT,
                                                   config::DataSourceNames::ZYRE };

    config::ConfigParams config_params = config::ConfigFileReader::load("../../config/data_sources.yaml");
    std::string db_name = "logs";
    std::vector<std::string> group_names;
    std::map<std::string, std::vector<std::string>> variable_names;

    //we take the ros topic and variable names and add them to
    //group_names and variable_names respectively
    for (int i=0; i<config_params.ros.topics.size(); i++)
    {
        std::string ros_topic_name = config_params.ros.topics[i].name;
        if (ros_topic_name[0] == '/')
        {
            ros_topic_name = ros_topic_name.substr(1, ros_topic_name.size()-1);
        }
        std::replace(ros_topic_name.begin(), ros_topic_name.end(), '/', '_');
        std::string group_name = config::DataSourceNames::ROS + "_" + ros_topic_name;
        group_names.push_back(group_name);
        variable_names[group_name] = config_params.ros.topics[i].variable_names;
    }

    //we take the zmq topic and variable names and add them to
    //group_names and variable_names respectively
    for (int i=0; i<config_params.zmq.topics.size(); i++)
    {
        std::string group_name = config::DataSourceNames::ZMQ + "_" + config_params.zmq.topics[i].name;
        group_names.push_back(group_name);
        variable_names[group_name] = config_params.zmq.topics[i].variable_names;
    }

    for (int k=0; k<config_params.ethercat.size(); k++)
    {
        //we take the ethercat input data and variable names and add them to
        //group_names and variable_names respectively
        std::string ethercat_input_group_name = config::DataSourceNames::ETHERCAT + "_" + config_params.ethercat[k].input_data_name;
        group_names.push_back(ethercat_input_group_name);

        std::vector<std::string> ethercat_input_variable_names;
        for (uint32_t j=0; j<config_params.ethercat[k].number_of_slaves; j++)
        {
            std::string j_string = std::to_string(j);
            for (unsigned int i=0; i<config_params.ethercat[k].input_variable_names.size(); i++)
            {
                std::string variable_name = config_params.ethercat[k].input_variable_names[i] + j_string;
                ethercat_input_variable_names.push_back(variable_name);
            }
        }
        variable_names[ethercat_input_group_name] = ethercat_input_variable_names;

        //we take the ethercat output data and variable names and add them to
        //group_names and variable_names respectively
        std::string ethercat_output_group_name = config::DataSourceNames::ETHERCAT + "_" + config_params.ethercat[k].output_data_name;
        group_names.push_back(ethercat_output_group_name);

        std::vector<std::string> ethercat_output_variable_names;
        for (uint32_t j=0; j<config_params.ethercat[k].number_of_slaves; j++)
        {
            std::string j_string = std::to_string(j);
            for (unsigned int i=0; i<config_params.ethercat[k].output_variable_names.size(); i++)
            {
                std::string variable_name = config_params.ethercat[k].output_variable_names[i] + j_string;
                ethercat_output_variable_names.push_back(variable_name);
            }
        }
        variable_names[ethercat_output_group_name] = ethercat_output_variable_names;
    }

    for (int i=0; i<config_params.zyre.groups.size(); i++)
    {
        for (int j=0; j<config_params.zyre.message_types.size(); j++)
        {
            std::string group_name = config::DataSourceNames::ZYRE + "_" + config_params.zyre.groups[i]
                                                                   + "_" + config_params.zyre.message_types[j];
            group_names.push_back(group_name);
            variable_names[group_name].push_back(config_params.zyre.message_types[j]);
        }
    }

    std::shared_ptr<loggers::DataLogger> logger =
        std::make_shared<loggers::MongodbLogger>(db_name, config_params.default_params.split_database,
                config_params.default_params.max_database_size, group_names, variable_names);

    std::shared_ptr<data_retrieval::QueryInterfaceManager> query_interface_manager =
    std::make_shared<data_retrieval::QueryInterfaceManager>(data_source_names,
                                                            config_params.zyre,
                                                            logger);

    std::cout << "[" << config_params.zyre.node_name << "] Query interface initialised" << std::endl;
    signal(SIGINT, checkTermination);
    while (!terminate)
    {
        sleep(0.5);
    }

    return 0;
}
