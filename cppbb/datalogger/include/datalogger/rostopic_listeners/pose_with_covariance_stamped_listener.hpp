#ifndef POSE_WITH_COVARIANCE_STAMPED_LISTENER_H
#define POSE_WITH_COVARIANCE_STAMPED_LISTENER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "datalogger/rostopic_listeners/rostopic_listener_base.hpp"

namespace ros_listeners
{
    class PoseWithCovarianceStampedListener : public ROSTopicListenerBase
    {
    public:
        PoseWithCovarianceStampedListener(const std::string &topic_name, const std::string &topic_type,
                const std::vector<std::string> &variable_names, double max_frequency,
                std::shared_ptr<loggers::DataLogger> data_logger);
        virtual ~PoseWithCovarianceStampedListener();

        virtual void createSubscriber(std::shared_ptr<ros::NodeHandle> nh);
        virtual void log(const topic_tools::ShapeShifter::ConstPtr &msg);
    };
}

#endif /* POSE_WITH_COVARIANCE_STAMPED_LISTENER_H */
