#ifndef PATH_LISTENER_HPP
#define PATH_LISTENER_HPP

#include <ros/ros.h>
#include <nav_msgs/Path.h>

#include "datalogger/rostopic_listeners/rostopic_listener_base.hpp"

namespace ros_listeners
{
    class PathListener : public ROSTopicListenerBase
    {
    public:
        PathListener(const std::string &topic_name, const std::string &topic_type, 
                const std::vector<std::string> &variable_names, double max_frequency,
                std::shared_ptr<loggers::DataLogger> data_logger);
        virtual ~PathListener();

        virtual void createSubscriber(std::shared_ptr<ros::NodeHandle> nh);
        virtual void log(const topic_tools::ShapeShifter::ConstPtr &msg);
    };
}

#endif /* PATH_LISTENER_HPP */
