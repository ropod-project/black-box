#ifndef TWIST_LISTENER_HPP
#define TWIST_LISTENER_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "datalogger/rostopic_listeners/rostopic_listener_base.hpp"

namespace ros_listeners
{
    class TwistListener : public ROSTopicListenerBase
    {
    public:
        TwistListener(const std::string &topic_name, const std::string &topic_type,
                const std::vector<std::string> &variable_names, double max_frequency,
                std::shared_ptr<loggers::DataLogger> data_logger);
        virtual ~TwistListener();

        virtual void createSubscriber(std::shared_ptr<ros::NodeHandle> nh);
        virtual void log(const topic_tools::ShapeShifter::ConstPtr &msg);
    };
}

#endif /* TWIST_LISTENER_HPP */
