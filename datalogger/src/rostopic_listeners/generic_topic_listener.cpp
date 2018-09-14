#include "datalogger/rostopic_listeners/generic_topic_listener.hpp"

namespace ros_listeners
{
    GenericTopicListener::GenericTopicListener(const std::string &topic_name, const std::string &topic_type, std::shared_ptr<loggers::DataLogger> data_logger)
        : ROSTopicListenerBase(topic_name, topic_type, data_logger) { }

    GenericTopicListener::~GenericTopicListener() { }

    void GenericTopicListener::createSubscriber(std::shared_ptr<ros::NodeHandle> nh)
    {
        is_registered_type_set_ = false;
        subscriber_.reset(new ros::Subscriber(
                    nh->subscribe<topic_tools::ShapeShifter>
                        (topic_name_, 1, boost::bind(&GenericTopicListener::log, this, _1))));
    }

    void GenericTopicListener::log(const topic_tools::ShapeShifter::ConstPtr &msg)
    {
        std::vector<float> values;

        auto& datatype_name = msg->getDataType();

        if(!is_registered_type_set_)
        {
            registered_type_ =
                RosIntrospection::buildROSTypeMapFromDefinition(datatype_name, msg->getMessageDefinition() );
            is_registered_type_set_ = true;
        }
        std::vector<uint8_t> buffer( msg->size() );
        ros::serialization::OStream stream(&buffer[0], msg->size() * sizeof(uint8_t));
        msg->write(stream);

        uint8_t* buffer_ptr = &buffer[0];

        RosIntrospection::SString topicname( topic_name_.data(), topic_name_.length() );

        RosIntrospection::ROSTypeFlat flat_container;
        RosIntrospection::buildRosFlatType( registered_type_,
                          RosIntrospection::ROSType(datatype_name),
                          topicname,
                          buffer_ptr,
                          &flat_container
                        );

        // integers/doubles etc.
        for(auto& it: flat_container.value) {
            RosIntrospection::SString field_name;
            it.first.toStr(field_name);
            values.push_back((float)(double)it.second);
        }

        // strings
        for(auto& it: flat_container.name) {
            RosIntrospection::SString field_name;
            it.first.toStr(field_name);
        }

        // TODO: check if message has a header and use timestamp from there instead
        data_tunnel_interface_.tunnelData(readers::DataReader::formatVariableName(config::DataSourceNames::ROS, topic_name_),
                                          getTimestamp(),
                                          values);
    }
}
