#ifndef WRENCH_STAMPED_LISTENER_H
#define WRENCH_STAMPED_LISTENER_H

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

#include "datalogger/rostopic_listeners/rostopic_listener_base.hpp"

namespace ros_listeners
{
    class WrenchStampedListener : public ROSTopicListenerBase
    {
    public:
        WrenchStampedListener(const std::string &topic_name, const std::string &topic_type,
                const std::vector<std::string> &variable_names, double max_frequency,
                std::shared_ptr<loggers::DataLogger> data_logger);
        virtual ~WrenchStampedListener();

        virtual void createSubscriber(std::shared_ptr<ros::NodeHandle> nh);
        virtual void log(const topic_tools::ShapeShifter::ConstPtr &msg);
    };
}

#endif /* WRENCH_STAMPED_LISTENER_H */
