#include "config/config_file_reader.hpp"

namespace config
{
    /**
     * Loads the configuration parameters of a black box from the given YAML file
     *
     * @param config_file_name absolute path of a config file
     */
    ConfigParams ConfigFileReader::load(const std::string config_file_name)
    {
        ConfigParams params;
        YAML::Node root;
        try
        {
            root = YAML::LoadFile(config_file_name);
        }
        catch (const std::exception& e)
        {
            std::cout << e.what() << "\n";
            return params;
        }

        for (YAML::const_iterator it=root.begin(); it != root.end(); ++it)
        {
            std::string name = it->begin()->first.as<std::string>();
            YAML::Node node = it->begin()->second;
            if (name == ConfigKeys::DEFAULT_PARAMETERS)
            {
                if (node["max_frequency"])
                {
                    params.default_params.max_frequency = node["max_frequency"].as<double>();
                }
                else
                {
                    throw ConfigException("default_params: max_frequency not specified");
                }
                if (node["max_database_size"])
                {
                    params.default_params.max_database_size = node["max_database_size"].as<double>();
                }
                else
                {
                    throw ConfigException("default_params: max_database_size not specified");
                }
                if (node["split_database"])
                {
                    params.default_params.split_database = node["split_database"].as<bool>();
                }
                else
                {
                    throw ConfigException("default_params: split_database not specified");
                }
            }
            else if (name == ConfigKeys::ROS)
            {
                if (node["ros_master_uri"])
                {
                    params.ros.ros_master_uri = node["ros_master_uri"].as<std::string>();
                }
                else
                {
                    throw ConfigException("ros: ros_master_uri not specified");
                }

                if (node["topics"])
                {
                    for (YAML::const_iterator ros_topic_it=node["topics"].begin(); ros_topic_it != node["topics"].end(); ++ros_topic_it)
                    {
                        YAML::Node params_node = ros_topic_it->begin()->second;
                        RosTopicParams topic_params;

                        if (params_node["name"])
                        {
                            topic_params.name = params_node["name"].as<std::string>();
                        }
                        else
                        {
                            throw ConfigException("ros: ROS topic name not specified");
                        }

                        if (params_node["type"])
                        {
                            topic_params.type = params_node["type"].as<std::string>();
                        }
                        else
                        {
                            throw ConfigException("ros: ROS topic type not specified");
                        }

                        if (params_node["variable_names"])
                        {
                            topic_params.variable_names = params_node["variable_names"].as<std::vector<std::string>>();
                        }

                        if (params_node["max_frequency"])
                        {
                            topic_params.max_frequency = params_node["max_frequency"].as<double>();
                        }
                        else
                        {
                            topic_params.max_frequency = params.default_params.max_frequency;
                        }

                        params.ros.topics.push_back(topic_params);
                    }
                }
            }
            else if (name == ConfigKeys::ZMQ)
            {

                if (node["topics"])
                {
                    for (YAML::const_iterator zmq_topic_it=node["topics"].begin(); zmq_topic_it != node["topics"].end(); ++zmq_topic_it)
                    {
                        YAML::Node params_node = zmq_topic_it->begin()->second;
                        ZmqTopicParams topic_params;

                        if (params_node["url"])
                        {
                            topic_params.url = params_node["url"].as<std::string>();
                        }
                        else
                        {
                            throw ConfigException("zmq: url not specified");
                        }

                        if (params_node["port"])
                        {
                            topic_params.port = params_node["port"].as<int>();
                        }
                        else
                        {
                            throw ConfigException("zmq: port not specified");
                        }

                        if (params_node["name"])
                        {
                            topic_params.name = params_node["name"].as<std::string>();
                        }
                        else
                        {
                            // topic name is not mandatory
                            topic_params.name = "";
                        }

                        if (params_node["type"])
                        {
                            topic_params.type = params_node["type"].as<std::string>();
                        }
                        else
                        {
                            throw ConfigException("zmq: ZMQ topic type not specified");
                        }

                        if (params_node["variable_names"])
                        {
                            topic_params.variable_names = params_node["variable_names"].as<std::vector<std::string>>();
                        }

                        if (params_node["max_frequency"])
                        {
                            topic_params.max_frequency = params_node["max_frequency"].as<double>();
                        }
                        else
                        {
                            topic_params.max_frequency = params.default_params.max_frequency;
                        }

                        params.zmq.topics.push_back(topic_params);
                    }
                }
            }
            else if (name == ConfigKeys::ZYRE)
            {
                params.zyre.node_name = node["name"].as<std::string>();
                params.zyre.groups = node["groups"].as<std::vector<std::string>>();
                params.zyre.message_types = node["message_types"].as<std::vector<std::string>>();
            }
            else if (name == ConfigKeys::ETHERCAT)
            {
                for (YAML::const_iterator eth_interface_it=node["interfaces"].begin(); eth_interface_it != node["interfaces"].end(); ++eth_interface_it)
                {
                    YAML::Node params_node = eth_interface_it->begin()->second;
                    EthercatParams ethercat_params;

                    if (params_node["interface"])
                    {
                        ethercat_params.interface = params_node["interface"].as<std::string>();
                    }
                    else
                    {
                        throw ConfigException("ethercat: interface not specified");
                    }

                    if (params_node["number_of_slaves"])
                    {
                        ethercat_params.number_of_slaves = params_node["number_of_slaves"].as<uint32_t>();
                    }
                    else
                    {
                        throw ConfigException("ethercat: number_of_slaves not specified");
                    }

                    if (params_node["input_message_size"])
                    {
                        ethercat_params.input_message_size = params_node["input_message_size"].as<uint32_t>();
                    }
                    else
                    {
                        throw ConfigException("ethercat: input_message_size not specified");
                    }

                    if (params_node["output_message_size"])
                    {
                        ethercat_params.output_message_size = params_node["output_message_size"].as<uint32_t>();
                    }
                    else
                    {
                        throw ConfigException("ethercat: output_message_size not specified");
                    }

                    if (params_node["output_message_elements"])
                    {
                        for (YAML::const_iterator ethercat_it=params_node["output_message_elements"].begin(); ethercat_it != params_node["output_message_elements"].end(); ++ethercat_it)
                        {
                            YAML::Node output_params_node = ethercat_it->begin()->second;
                            EthercatMessageElement message_element;

                            if (output_params_node["name"])
                            {
                                message_element.name = output_params_node["name"].as<std::string>();
                            }
                            else
                            {
                                throw ConfigException("ethercat: output message element name not specified");
                            }

                            if (output_params_node["type"])
                            {
                                message_element.type = output_params_node["type"].as<std::string>();
                            }
                            else
                            {
                                throw ConfigException("ethercat: output message element type not specified");
                            }

                            ethercat_params.output_message_elements.push_back(message_element);
                        }
                    }

                    if (params_node["input_message_elements"])
                    {
                        for (YAML::const_iterator ethercat_it=params_node["input_message_elements"].begin(); ethercat_it != params_node["input_message_elements"].end(); ++ethercat_it)
                        {
                            YAML::Node input_params_node = ethercat_it->begin()->second;
                            EthercatMessageElement message_element;

                            if (input_params_node["name"])
                            {
                                message_element.name = input_params_node["name"].as<std::string>();
                            }
                            else
                            {
                                throw ConfigException("ethercat: input message element name not specified");
                            }

                            if (input_params_node["type"])
                            {
                                message_element.type = input_params_node["type"].as<std::string>();
                            }
                            else
                            {
                                throw ConfigException("ethercat: input message element type not specified");
                            }

                            ethercat_params.input_message_elements.push_back(message_element);
                        }
                    }

                    if (params_node["output_data_items"])
                    {
                        ethercat_params.output_data_name = params_node["output_data_items"]["name"].as<std::string>();
                        ethercat_params.output_variable_names = params_node["output_data_items"]["variable_names"].as<std::vector<std::string>>();
                    }

                    if (params_node["input_data_items"])
                    {
                        ethercat_params.input_data_name = params_node["input_data_items"]["name"].as<std::string>();
                        ethercat_params.input_variable_names = params_node["input_data_items"]["variable_names"].as<std::vector<std::string>>();
                    }

                    params.ethercat.push_back(ethercat_params);
                }
            }
        }

        return params;
    }
}
