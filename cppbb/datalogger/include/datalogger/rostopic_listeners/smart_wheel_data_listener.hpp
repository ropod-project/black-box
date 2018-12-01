#ifndef SMART_WHEEL_DATA_LISTENER_HPP
#define SMART_WHEEL_DATA_LISTENER_HPP

#include <ros/ros.h>
#include <ropod_ros_msgs/SmartWheelData.h>

#include "datalogger/rostopic_listeners/rostopic_listener_base.hpp"

namespace ros_listeners
{
    class SmartWheelDataListener : public ROSTopicListenerBase
    {
    public:
        SmartWheelDataListener(const std::string &topic_name, const std::string &topic_type, 
                const std::vector<std::string> &variable_names, double max_frequency,
                std::shared_ptr<loggers::DataLogger> data_logger);
        virtual ~SmartWheelDataListener();

        virtual void createSubscriber(std::shared_ptr<ros::NodeHandle> nh);
        virtual void log(const topic_tools::ShapeShifter::ConstPtr &msg);
    };
}

#endif /* SMART_WHEEL_DATA_LISTENER_HPP */
