#ifndef GENERIC_TOPIC_LISTENER_H
#define GENERIC_TOPIC_LISTENER_H

#include <ros/ros.h>
#include <ros_type_introspection/ros_introspection.hpp>
#include <json/json.h>

#include "datalogger/rostopic_listeners/rostopic_listener_base.hpp"

namespace ros_listeners
{
    class GenericTopicListener : public ROSTopicListenerBase
    {
    public:
        GenericTopicListener(const std::string &topic_name, const std::string &topic_type, int max_frequency, std::shared_ptr<loggers::DataLogger> data_logger);
        virtual ~GenericTopicListener();

        virtual void createSubscriber(std::shared_ptr<ros::NodeHandle> nh);
        virtual void log(const topic_tools::ShapeShifter::ConstPtr &msg);

    private:
        std::string getJsonDocument(const RosIntrospection::FlatMessage& flat_container,
                                    const RosIntrospection::RenamedValues& renamed_values);
        std::string removeTopicFromVariableName(const std::string& variable_name);

        RosIntrospection::Parser ros_type_parser;
        Json::StreamWriterBuilder json_stream_builder;
        bool is_registered_type_set;
    };
}

#endif
