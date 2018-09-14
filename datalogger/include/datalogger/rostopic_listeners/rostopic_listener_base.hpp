#ifndef ROSTOPIC_LISTENER_BASE_H
#define ROSTOPIC_LISTENER_BASE_H

#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

#include "datalogger/data_readers/data_reader.hpp"
#include "datalogger/data_readers/data_tunnel_interface.h"
#include "datalogger/data_loggers/data_logger.hpp"

#include <sstream>

namespace ros_listeners
{
    class ROSTopicListenerBase
    {
    public:
        ROSTopicListenerBase(const std::string &topic_name, const std::string &topic_type, const std::vector<std::string> &variable_names, double max_frequency, std::shared_ptr<loggers::DataLogger> data_logger);
        virtual ~ROSTopicListenerBase();

        virtual void createSubscriber(std::shared_ptr<ros::NodeHandle> nh) = 0;
        virtual void log(const topic_tools::ShapeShifter::ConstPtr &msg) = 0;
        void shutdownSubscriber();
        void resetSubscriber();
    protected:
        double getTimestamp();
        double getTimestamp(ros::Time time);
        /*
         * Returns true if time since last message exceeds minimum_period
         */
        bool isTimeElapsed();

        std::shared_ptr<ros::Subscriber> subscriber_;
        std::string topic_name_;
        std::string topic_type_;
        std::vector<std::string> variable_names_;
        double max_frequency_;
        ros::Duration minimum_period_;
        ros::Time previous_msg_time_;
        readers::DataTunnelInterface data_tunnel_interface_;
    };
}

#endif /* ROSTOPIC_LISTENER_BASE_H */
