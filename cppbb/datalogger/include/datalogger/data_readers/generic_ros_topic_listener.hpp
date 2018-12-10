#ifndef GENERIC_TOPIC_LISTENER_H
#define GENERIC_TOPIC_LISTENER_H

#include <chrono>
#include <ros/ros.h>
#include <ros_type_introspection/ros_introspection.hpp>
#include <topic_tools/shape_shifter.h>
#include <json/json.h>

#include "datalogger/data_readers/data_reader.hpp"
#include "datalogger/data_readers/data_tunnel_interface.h"
#include "datalogger/data_loggers/data_logger.hpp"

namespace readers
{
    class GenericTopicListener
    {
    public:
        GenericTopicListener(const std::string &topic_name, const std::string &topic_type, int max_frequency, std::shared_ptr<loggers::DataLogger> data_logger);
        ~GenericTopicListener();

        void createSubscriber(std::shared_ptr<ros::NodeHandle> nh);
        void log(const topic_tools::ShapeShifter::ConstPtr &msg);
        void shutdownSubscriber();
        void resetSubscriber();
    private:
        double getTimestamp();
        double getTimestamp(ros::Time time);
        /*
         * Returns true if time since last message exceeds minimum_period
         */
        bool minTimeElapsed();


        std::string getJsonDocument(const RosIntrospection::FlatMessage& flat_container,
                                    const RosIntrospection::RenamedValues& renamed_values);
        std::string removeTopicFromVariableName(const std::string& variable_name);

        std::shared_ptr<ros::Subscriber> subscriber_;
        std::string topic_name_;
        std::string topic_type_;
        double max_frequency_;
        ros::Duration min_time_between_msgs_;
        ros::Time previous_msg_time_;
        readers::DataTunnelInterface data_tunnel_interface_;

        RosIntrospection::Parser ros_type_parser;
        Json::StreamWriterBuilder json_stream_builder;
        bool is_registered_type_set;
    };
}

#endif
