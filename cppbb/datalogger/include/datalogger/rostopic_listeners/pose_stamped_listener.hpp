#ifndef POSE_STAMPED_LISTENER_H
#define POSE_STAMPED_LISTENER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "datalogger/rostopic_listeners/rostopic_listener_base.hpp"

namespace ros_listeners
{
    class PoseStampedListener : public ROSTopicListenerBase
    {
    public:
        PoseStampedListener(const std::string &topic_name, const std::string &topic_type,
                const std::vector<std::string> &variable_names, double max_frequency,
                std::shared_ptr<loggers::DataLogger> data_logger);
        virtual ~PoseStampedListener();

        virtual void createSubscriber(std::shared_ptr<ros::NodeHandle> nh);
        virtual void log(const topic_tools::ShapeShifter::ConstPtr &msg);
    };
}

#endif /* POSE_STAMPED_LISTENER_H */
