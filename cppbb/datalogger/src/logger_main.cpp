#include <thread>
#include <chrono>

#include "config/config_enums.hpp"
#include "config/config_params.hpp"
#include "config/config_file_reader.hpp"

#include "datalogger/data_readers/ethercat_reader.hpp"
#include "datalogger/data_readers/zyre_reader.hpp"
#include "datalogger/data_readers/rostopic_reader.hpp"
#include "datalogger/data_readers/json_zmq_reader.hpp"

#include "datalogger/data_loggers/mongodb_logger.hpp"
#include "datalogger/data_loggers/text_logger.hpp"

int main(int argc, char *argv[])
{
    std::vector<std::string> data_source_names = { config::DataSourceNames::ROS,
                                                   config::DataSourceNames::ZMQ,
                                                   config::DataSourceNames::ETHERCAT,
                                                   config::DataSourceNames::ZYRE };

    if (argc < 2)
    {
        std::cout << "Usage: ./black_box_logger <absolute-path-to-black-box-config-file>";
        return 0;
    }

    std::string black_box_config_file(argv[1]);

    std::cout << "Reading black box configuration file " << black_box_config_file << std::endl;
    config::ConfigParams config_params = config::ConfigFileReader::load(black_box_config_file);
    std::cout << "[" << config_params.zyre.node_name << "] Configuring black box" << std::endl;

    std::string db_name_prefix = "logs";
    std::vector<std::string> group_names;
    std::map<std::string, std::vector<std::string>> variable_names;

    //we take the ros topic and variable names and add them to
    //group_names and variable_names respectively
    for (int i=0; i<config_params.ros.topics.size(); i++)
    {
        std::string group_name = readers::DataReader::formatVariableName(config::DataSourceNames::ROS, config_params.ros.topics[i].name);
        group_names.push_back(group_name);
        variable_names[group_name] = config_params.ros.topics[i].variable_names;
    }

    //we take the zmq topic and variable names and add them to
    //group_names and variable_names respectively
    for (int i=0; i<config_params.zmq.topics.size(); i++)
    {
        std::string group_name = readers::DataReader::formatVariableName(config::DataSourceNames::ZMQ, config_params.zmq.topics[i].name);
        group_names.push_back(group_name);
        variable_names[group_name] = config_params.zmq.topics[i].variable_names;
    }

    for (int k=0; k<config_params.ethercat.size(); k++)
    {
        //we take the ethercat input data and variable names and add them to
        //group_names and variable_names respectively
        std::string ethercat_input_group_name = readers::DataReader::formatVariableName(config::DataSourceNames::ETHERCAT, config_params.ethercat[k].input_data_name);
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
        std::string ethercat_output_group_name = readers::DataReader::formatVariableName(config::DataSourceNames::ETHERCAT, config_params.ethercat[k].output_data_name);
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
            std::string group_name = readers::DataReader::formatVariableName(
                    config::DataSourceNames::ZYRE,
                    config_params.zyre.groups[i] + "_" + config_params.zyre.message_types[j]);
            group_names.push_back(group_name);
            variable_names[group_name].push_back(config_params.zyre.message_types[j]);
        }
    }

    bool debug = false;
    if (debug)
    {
        //we print the contents of group_names and variable_names for debugging purposes
        for(unsigned int i=0; i<group_names.size(); i++)
        {
            std::string name = group_names[i];
            std::vector<std::string> var_names = variable_names[name];
            std::cout << name << ": ";
            for (int j=0; j<var_names.size(); j++)
            {
                std::cout << var_names[j] << " ";
            }
            std::cout << std::endl;
        }
    }

    std::shared_ptr<loggers::DataLogger> logger =
        std::make_shared<loggers::MongodbLogger>(db_name_prefix, config_params.default_params.split_database,
                config_params.default_params.max_database_size, group_names, variable_names);

    // std::shared_ptr<EthercatReader> ethercat_reader = std::make_shared<EthercatReader>(config_params.ethercat[0], logger);
    // ethercat_reader->start();

    // std::vector<std::shared_ptr<ZMQReader> > zmq_readers;
    // for (unsigned int i=0; i<config_params.zmq.topics.size(); i++)
    // {
    //     std::shared_ptr<ZMQReader> zmq_reader = std::make_shared<JsonZMQReader>(config_params.zmq.topics[i], logger);
    //     zmq_readers.push_back(zmq_reader);
    //     zmq_reader->start();
    // }
    std::shared_ptr<readers::ZyreReader> zyre_reader =
    std::make_shared<readers::ZyreReader>(config_params.zyre, logger);

    // this last output is before configuring the ROS topic reader because that one will block the execution
    std::cout << "[" << config_params.zyre.node_name << "] Logger configured; ready to log data" << std::endl;

    readers::ROSTopicReader ros_topic_reader(argc, argv, "rostopic_reader",
                    config_params.ros, config_params.default_params.max_frequency, logger);
    ros_topic_reader.start();
    ros_topic_reader.loop();

    return 0;
}
