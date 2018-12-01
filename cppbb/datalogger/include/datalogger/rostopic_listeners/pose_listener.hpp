#ifndef POSE_LISTENER_H
#define POSE_LISTENER_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include "datalogger/rostopic_listeners/rostopic_listener_base.hpp"

namespace ros_listeners
{
    class PoseListener : public ROSTopicListenerBase
    {
    public:
        PoseListener(const std::string &topic_name, const std::string &topic_type,
                const std::vector<std::string> &variable_names, double max_frequency,
                std::shared_ptr<loggers::DataLogger> data_logger);
        virtual ~PoseListener();

        virtual void createSubscriber(std::shared_ptr<ros::NodeHandle> nh);
        virtual void log(const topic_tools::ShapeShifter::ConstPtr &msg);
    };
}

#endif /* POSE_LISTENER_H */
