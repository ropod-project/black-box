#include "datalogger/rostopic_listeners/generic_topic_listener.hpp"

namespace ros_listeners
{
    GenericTopicListener::GenericTopicListener(const std::string &topic_name, const std::string &topic_type, int max_frequency, std::shared_ptr<loggers::DataLogger> data_logger)
        : ROSTopicListenerBase(topic_name, topic_type, max_frequency, data_logger) { }

    GenericTopicListener::~GenericTopicListener() { }

    void GenericTopicListener::createSubscriber(std::shared_ptr<ros::NodeHandle> nh)
    {
        this->is_registered_type_set = false;
        subscriber_.reset(new ros::Subscriber(
                    nh->subscribe<topic_tools::ShapeShifter>
                        (topic_name_, 1, boost::bind(&GenericTopicListener::log, this, _1))));
    }

    void GenericTopicListener::log(const topic_tools::ShapeShifter::ConstPtr &msg)
    {
        if (this->isTimeElapsed())
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

    std::string GenericTopicListener::getJsonDocument(const RosIntrospection::FlatMessage& flat_container,
                                                      const RosIntrospection::RenamedValues& renamed_values)
    {
        Json::Value variable_dict;
        variable_dict["timestamp"] = this->getTimestamp();

        // we store the numerical values from the message
        for (auto value_it: renamed_values)
        {
            std::string full_variable_name = this->removeTopicFromVariableName(value_it.first);
            std::string name_component = "";
            std::vector<unsigned int> slash_indices = { 0 };
            bool is_array_element = false;

            Json::Value* current_variable_dict = &variable_dict;
            for (unsigned int i=0; i<full_variable_name.size(); i++)
            {
                if (full_variable_name[i] == '/')
                {
                    // we add one to the index because we want to start
                    // the next segment of the name from the character
                    // directly after the slash
                    slash_indices.push_back(i+1);

                    // e.g. assuming 'full_variable_name' is "variable/map",
                    // the slash index will be 9, so 'name_component' will
                    // get the value "variable", which corresponds to
                    // full_variable_name[0] : full_variable_name[8] (inclusive)
                    name_component = full_variable_name.substr(slash_indices[slash_indices.size()-2],
                                                               slash_indices[slash_indices.size()-1] - slash_indices[slash_indices.size()-2] - 1);
                    if (!current_variable_dict->isMember(name_component))
                    {
                        (*current_variable_dict)[name_component] = Json::Value();
                    }
                    current_variable_dict = &(*current_variable_dict)[name_component];
                }
                else if (full_variable_name[i] == '.')
                {
                    is_array_element = true;

                    // if we have an array element, 'full_variable_name' may for instance be
                    // "variable/map.0"; in this example, the last slash index will be 9 and 'i' will
                    // have the value 12, so 'name_component'  will get the value "map",
                    // which corresponds to full_variable_name[9:11] (inclusive)
                    name_component = full_variable_name.substr(slash_indices[slash_indices.size()-1],
                                                               i - slash_indices[slash_indices.size()-1]);

                    if (!current_variable_dict->isMember(name_component))
                    {
                        (*current_variable_dict)[name_component] = Json::Value();
                    }

                    const RosIntrospection::Variant& value = value_it.second;
                    (*current_variable_dict)[name_component].append(value.convert<double>());
                }
            }

            if (!is_array_element)
            {
                // again assuming that 'full_variable_name' is "variable/map",
                // the last slash index will be 9, so 'name_component' will
                // get the value "map", which corresponds to
                // full_variable_name[9] : full_variable_name[11] (inclusive)
                name_component = full_variable_name.substr(slash_indices[slash_indices.size()-1],
                                                           full_variable_name.size() - slash_indices[slash_indices.size()-1]);

                const RosIntrospection::Variant& value = value_it.second;
                (*current_variable_dict)[name_component] = value.convert<double>();
            }
        }

        // we store the string values from the message
        for (auto value_it : flat_container.name)
        {
            std::string full_variable_name = this->removeTopicFromVariableName(value_it.first.toStdString());
            std::string name_component = "";
            std::vector<unsigned int> slash_indices = { 0 };
            bool is_array_element = false;

            Json::Value* current_variable_dict = &variable_dict;
            for (unsigned int i=0; i<full_variable_name.size(); i++)
            {
                if (full_variable_name[i] == '/')
                {
                    // we add one to the index because we want to start
                    // the next segment of the name from the character
                    // directly after the slash
                    slash_indices.push_back(i+1);

                    // e.g. assuming 'full_variable_name' is "variable/map",
                    // the slash index will be 9, so 'name_component' will
                    // get the value "variable", which corresponds to
                    // full_variable_name[0] : full_variable_name[8] (inclusive)
                    name_component = full_variable_name.substr(slash_indices[slash_indices.size()-2],
                                                               slash_indices[slash_indices.size()-1] - slash_indices[slash_indices.size()-2] - 1);
                    if (!current_variable_dict->isMember(name_component))
                    {
                        (*current_variable_dict)[name_component] = Json::Value();
                    }
                    current_variable_dict = &(*current_variable_dict)[name_component];
                }
                else if (full_variable_name[i] == '.')
                {
                    is_array_element = true;

                    // again assuming that 'full_variable_name' is "variable/map",
                    // the last slash index will be 9, so 'name_component' will
                    // get the value "map", which corresponds to
                    // full_variable_name[9] : full_variable_name[11] (inclusive)
                    name_component = full_variable_name.substr(slash_indices[slash_indices.size()-1],
                                                               i - slash_indices[slash_indices.size()-1]);

                    if (!current_variable_dict->isMember(name_component))
                    {
                        (*current_variable_dict)[name_component] = Json::Value();
                    }

                    const RosIntrospection::Variant& value = value_it.second;
                    (*current_variable_dict)[name_component].append(value.convert<double>());
                }
            }

            if (!is_array_element)
            {
                // again assuming that 'full_variable_name' is "variable/map",
                // the last slash index will be 9, so 'name_component' will
                // get the value "map", which corresponds to
                // full_variable_name[9] : full_variable_name[11] (inclusive)
                name_component = full_variable_name.substr(slash_indices[slash_indices.size()-1],
                                                           full_variable_name.size() - slash_indices[slash_indices.size()-1]);

                (*current_variable_dict)[name_component] = value_it.second;
            }
        }

        const std::string json_str = Json::writeString(this->json_stream_builder, variable_dict);
        return json_str;
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
