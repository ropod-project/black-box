#ifndef STRING_LISTENER_H
#define STRING_LISTENER_H

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "datalogger/rostopic_listeners/rostopic_listener_base.hpp"

namespace ros_listeners
{
    class StringListener : public ROSTopicListenerBase
    {
    public:
        StringListener(const std::string &topic_name, const std::string &topic_type, 
                const std::vector<std::string> &variable_names, double max_frequency,
                std::shared_ptr<loggers::DataLogger> data_logger);
        virtual ~StringListener();

        virtual void createSubscriber(std::shared_ptr<ros::NodeHandle> nh);
        virtual void log(const topic_tools::ShapeShifter::ConstPtr &msg);
    };
}

#endif /* STRING_LISTENER_H */
