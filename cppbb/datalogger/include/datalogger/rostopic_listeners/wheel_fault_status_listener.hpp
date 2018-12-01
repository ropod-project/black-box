#ifndef WHEEL_FAULT_STATUS_LISTENER_HPP
#define WHEEL_FAULT_STATUS_LISTENER_HPP

#include <ros/ros.h>
#include <youbot_driver_ros_interface/WheelFaultStatus.h>

#include "datalogger/rostopic_listeners/rostopic_listener_base.hpp"

namespace ros_listeners
{
    class WheelFaultStatusListener : public ROSTopicListenerBase
    {
    public:
        WheelFaultStatusListener(const std::string &topic_name, const std::string &topic_type,
                const std::vector<std::string> &variable_names, double max_frequency,
                std::shared_ptr<loggers::DataLogger> data_logger);
        virtual ~WheelFaultStatusListener();

        virtual void createSubscriber(std::shared_ptr<ros::NodeHandle> nh);
        virtual void log(const topic_tools::ShapeShifter::ConstPtr &msg);
    };
}

#endif /* WHEEL_FAULT_STATUS_LISTENER_HPP */
