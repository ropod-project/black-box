#ifndef LASERSCAN_LISTENER_HPP
#define LASERSCAN_LISTENER_HPP

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include "datalogger/rostopic_listeners/rostopic_listener_base.hpp"

namespace ros_listeners
{
    class LaserScanListener : public ROSTopicListenerBase
    {
    public:
        LaserScanListener(const std::string &topic_name, const std::string &topic_type,
                const std::vector<std::string> &variable_names, double max_frequency,
                std::shared_ptr<loggers::DataLogger> data_logger);
        virtual ~LaserScanListener();

        virtual void createSubscriber(std::shared_ptr<ros::NodeHandle> nh);
        virtual void log(const topic_tools::ShapeShifter::ConstPtr &msg);
    };
}

#endif /* LASERSCAN_LISTENER_HPP */
