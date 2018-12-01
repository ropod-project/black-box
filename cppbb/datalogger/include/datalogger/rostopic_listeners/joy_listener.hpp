#ifndef JOY_LISTENER_HPP
#define JOY_LISTENER_HPP

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include "datalogger/rostopic_listeners/rostopic_listener_base.hpp"

namespace ros_listeners
{
    class JoyListener : public ROSTopicListenerBase
    {
    public:
        JoyListener(const std::string &topic_name, const std::string &topic_type,
                const std::vector<std::string> &variable_names, double max_frequency,
                std::shared_ptr<loggers::DataLogger> data_logger);
        virtual ~JoyListener();

        virtual void createSubscriber(std::shared_ptr<ros::NodeHandle> nh);
        virtual void log(const topic_tools::ShapeShifter::ConstPtr &msg);
    };
}

#endif /* JOY_LISTENER_HPP */
