#ifndef WHEEL_VELOCITIES_LISTENER_HPP
#define WHEEL_VELOCITIES_LISTENER_HPP

#include <ros/ros.h>
#include <youbot_driver_ros_interface/WheelVelocities.h>

#include "datalogger/rostopic_listeners/rostopic_listener_base.hpp"

namespace ros_listeners
{
    class WheelVelocitiesListener : public ROSTopicListenerBase
    {
    public:
        WheelVelocitiesListener(const std::string &topic_name, const std::string &topic_type,
                const std::vector<std::string> &variable_names, double max_frequency,
                std::shared_ptr<loggers::DataLogger> data_logger);
        virtual ~WheelVelocitiesListener();

        virtual void createSubscriber(std::shared_ptr<ros::NodeHandle> nh);
        virtual void log(const topic_tools::ShapeShifter::ConstPtr &msg);
    };
}

#endif /* WHEEL_VELOCITIES_LISTENER_HPP */
