#ifndef ODOM_LISTENER_H
#define ODOM_LISTENER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "datalogger/rostopic_listeners/rostopic_listener_base.hpp"

namespace ros_listeners
{
    class OdomListener : public ROSTopicListenerBase
    {
    public:
        OdomListener(const std::string &topic_name, const std::string &topic_type, 
                const std::vector<std::string> &variable_names, double max_frequency,
                std::shared_ptr<loggers::DataLogger> data_logger);
        virtual ~OdomListener();

        virtual void createSubscriber(std::shared_ptr<ros::NodeHandle> nh);
        virtual void log(const topic_tools::ShapeShifter::ConstPtr &msg);
    };
}

#endif /* ODOM_LISTENER_H */
