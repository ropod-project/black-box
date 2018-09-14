#ifndef DOOR_LISTENER_HPP
#define DOOR_LISTENER_HPP

#include <ros/ros.h>
#include <ropod_ros_msgs/ropod_door_detection.h>

#include "datalogger/rostopic_listeners/rostopic_listener_base.hpp"

namespace ros_listeners
{
    class DoorListener : public ROSTopicListenerBase
    {
    public:
        DoorListener(const std::string &topic_name, const std::string &topic_type,
                const std::vector<std::string> &variable_names, double max_frequency,
                std::shared_ptr<loggers::DataLogger> data_logger);
        virtual ~DoorListener();

        virtual void createSubscriber(std::shared_ptr<ros::NodeHandle> nh);
        virtual void log(const topic_tools::ShapeShifter::ConstPtr &msg);
    };
}

#endif /* DOOR_LISTENER_HPP */
