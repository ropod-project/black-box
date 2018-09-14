#ifndef JOINT_STATE_LISTENER_H
#define JOINT_STATE_LISTENER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "datalogger/rostopic_listeners/rostopic_listener_base.hpp"

namespace ros_listeners
{
    class JointStateListener : public ROSTopicListenerBase
    {
    public:
        JointStateListener(const std::string &topic_name, const std::string &topic_type,
                const std::vector<std::string> &variable_names, double max_frequency,
                std::shared_ptr<loggers::DataLogger> data_logger);
        virtual ~JointStateListener();

        virtual void createSubscriber(std::shared_ptr<ros::NodeHandle> nh);
        virtual void log(const topic_tools::ShapeShifter::ConstPtr &msg);
    };
}

#endif /* JOINT_STATE_LISTENER_H */
