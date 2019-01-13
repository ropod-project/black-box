#include "datalogger/data_readers/generic_ros_topic_listener.hpp"

namespace readers
{
    GenericTopicListener::GenericTopicListener(const std::string &topic_name, const std::string &topic_type, int max_frequency, std::shared_ptr<loggers::DataLogger> data_logger)
        : topic_name_(topic_name), topic_type_(topic_type),
          max_frequency_(max_frequency), data_tunnel_interface_(data_logger) { }

    GenericTopicListener::~GenericTopicListener()
    {
        this->subscriber_->shutdown();
    }

    void GenericTopicListener::createSubscriber(std::shared_ptr<ros::NodeHandle> nh)
    {
        this->is_registered_type_set = false;
        subscriber_.reset(new ros::Subscriber(
                    nh->subscribe<topic_tools::ShapeShifter>
                        (topic_name_, 1, boost::bind(&GenericTopicListener::log, this, _1))));
    }

    void GenericTopicListener::shutdownSubscriber()
    {
        this->subscriber_->shutdown();
    }

    void GenericTopicListener::resetSubscriber()
    {
        this->subscriber_.reset();
    }

    void GenericTopicListener::log(const topic_tools::ShapeShifter::ConstPtr &msg)
    {
        if (this->minTimeElapsed())
        {
            this->previous_msg_time_ = ros::Time::now();
            std::vector<float> values;
            if(!this->is_registered_type_set)
            {
                std::string datatype = msg->getDataType();
                std::string msg_definition = msg->getMessageDefinition();

                this->ros_type_parser.registerMessageDefinition(this->topic_name_,
                                                                RosIntrospection::ROSType(datatype),
                                                                msg_definition);
                this->is_registered_type_set = true;
            }

            std::vector<uint8_t> buffer;
            buffer.resize(msg->size());
            ros::serialization::OStream stream(buffer.data(), buffer.size());
            msg->write(stream);

            RosIntrospection::FlatMessage flat_container;
            RosIntrospection::RenamedValues renamed_values;
            this->ros_type_parser.deserializeIntoFlatContainer(this->topic_name_,
                                                               absl::Span<uint8_t>(buffer),
                                                               &flat_container,
                                                               100);
            this->ros_type_parser.applyNameTransform(this->topic_name_,
                                                     flat_container,
                                                     &renamed_values);

            std::string json_document = this->getJsonDocument(flat_container, renamed_values);
            data_tunnel_interface_.tunnelData(readers::DataReader::formatVariableName(config::DataSourceNames::ROS, topic_name_),
                                              json_document);
        }
    }

    double GenericTopicListener::getTimestamp()
    {
        auto now = std::chrono::system_clock::now();
        return std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() / 1000.0;
    }

    double GenericTopicListener::getTimestamp(ros::Time time)
    {
        return (time.toSec());
    }

    bool GenericTopicListener::minTimeElapsed()
    {
        ros::Time now = ros::Time::now();
        if (now - previous_msg_time_ > min_time_between_msgs_)
        {
            return true;
        }
        return false;
    }

    std::string GenericTopicListener::getJsonDocument(const RosIntrospection::FlatMessage& flat_container,
                                                      const RosIntrospection::RenamedValues& renamed_values)
    {
        Json::Value variable_dict;
        variable_dict["timestamp"] = this->getTimestamp();

        // we store the numerical values from the message
        for (auto value_it: renamed_values)
        {
            std::string full_variable_name = this->removeTopicFromVariableName(value_it.first);
            std::vector<std::string> tokens;
            std::string token;
            std::istringstream ss(full_variable_name);
            while (std::getline(ss, token, '/'))
            {
                if (!token.empty())
                {
                    tokens.push_back(token);
                }
            }
            Json::Value *current_value = &variable_dict;
            const RosIntrospection::Variant& value = value_it.second;
            double leaf_value = value.convert<double>();
            this->setLeafNode(tokens, current_value, static_cast<void *>(&leaf_value), Json::ValueType::realValue);
        }

        // we store the string values from the message
        for (auto value_it : flat_container.name)
        {
            std::string full_variable_name = this->removeTopicFromVariableName(value_it.first.toStdString());
            std::vector<std::string> tokens;
            std::string token;
            std::istringstream ss(full_variable_name);
            while (std::getline(ss, token, '/'))
            {
                if (!token.empty())
                {
                    tokens.push_back(token);
                }
            }
            Json::Value *current_value = &variable_dict;
            std::string leaf_value = value_it.second;
            this->setLeafNode(tokens, current_value, static_cast<void *>(&leaf_value), Json::ValueType::stringValue);
        }

        const std::string json_str = Json::writeString(this->json_stream_builder, variable_dict);
        return json_str;
    }

    // given a list of Json key names, sets the leaf_value
    // For example: given {tf, transforms.0, transform, translation, y}
    // it will set current_value["tf"]["transforms"][0]["transform"]["translation"]["y"] = leaf_value
    void GenericTopicListener::setLeafNode(const std::vector<std::string> &tree, Json::Value *current_value, void *leaf_value, Json::ValueType leaf_type)
    {
        for (int i = 0; i < tree.size(); i++)
        {
            std::size_t pos = tree[i].find('.');
            bool is_array = false;
            std::string array_name;
            int array_index;
            // array element (such as transforms.0)
            if (pos != std::string::npos)
            {
                array_name = tree[i].substr(0, pos);
                array_index = std::stoi(tree[i].substr(pos+1));
                is_array = true;
            }
            // if it's not an array element, create the next level in the tree
            if (!is_array)
            {
                if (!current_value->isMember(tree[i]))
                {
                    (*current_value)[tree[i]] = Json::Value();
                }
                current_value = &(*current_value)[tree[i]];
            }
            else
            {
                // check if array exists already in the Json element; if not, create it
                if (!current_value->isMember(array_name))
                {
                    (*current_value)[array_name] = Json::Value();
                }
                // if array element doesn't exist, append to the array
                if ((*current_value)[array_name].size() < array_index+1)
                {
                    Json::Value array_element;
                    // if this is not the leaf node
                    if (i < tree.size() - 1)
                    {
                        // call same function with remaining items in tree
                        setLeafNode(std::vector<std::string>(tree.begin()+i+1, tree.end()), &array_element, leaf_value, leaf_type);
                        (*current_value)[array_name].append(array_element);
                        return;
                    }
                    else // we're at at leaf node, we need to set its value. For example /odom/pose/covariance.0
                    {
                        (*current_value)[array_name].append(array_element);
                        if (leaf_type == Json::ValueType::realValue)
                        {
                            (*current_value)[array_name][array_index] = *(static_cast<double *>(leaf_value));
                        }
                        else if (leaf_type == Json::ValueType::stringValue)
                        {
                            (*current_value)[array_name][array_index] = *(static_cast<std::string *>(leaf_value));
                        }
                        return;
                    }
                }
                else
                {
                    // array element already exists, so just update it
                    Json::Value &array_element = (*current_value)[array_name][array_index];
                    setLeafNode(std::vector<std::string>(tree.begin()+i+1, tree.end()), &array_element, leaf_value, leaf_type);
                    return;
                }
            }
        }
        // set value of leaf
        if (leaf_type == Json::ValueType::realValue)
        {
            *current_value = *(static_cast<double *>(leaf_value));
        }
        else if( leaf_type == Json::ValueType::stringValue)
        {
            *current_value = *(static_cast<std::string *>(leaf_value));
        }
    }


    std::string GenericTopicListener::removeTopicFromVariableName(const std::string& variable_name)
    {
        // if, for example, variable_name is "/variable/map" and topic_name
        // is "/variable" (the lenght of topic_name is 9), the new variable name
        // will be variable_name[10:13] (inclusive), which is just "map"; similarly,
        // if variable_name is "variable/map" and topic_name is "variable"
        // (the lenght of topic_name is now 8), the new variable name will be
        // variable_name[9:12] (inclusive), which is again just "map"
        return variable_name.substr(this->topic_name_.size() + 1,
                                    variable_name.size() - this->topic_name_.size());
    }

}
