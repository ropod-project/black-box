#ifndef MOVE_BASE_RESULT_LISTENER_HPP
#define MOVE_BASE_RESULT_LISTENER_HPP

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseActionResult.h>

#include "datalogger/rostopic_listeners/rostopic_listener_base.hpp"

namespace ros_listeners
{
    class MoveBaseResultListener : public ROSTopicListenerBase
    {
    public:
        MoveBaseResultListener(const std::string &topic_name, const std::string &topic_type,
                const std::vector<std::string> &variable_names, double max_frequency,
                std::shared_ptr<loggers::DataLogger> data_logger);
        virtual ~MoveBaseResultListener();

        virtual void createSubscriber(std::shared_ptr<ros::NodeHandle> nh);
        virtual void log(const topic_tools::ShapeShifter::ConstPtr &msg);
    };
}

#endif /* MOVE_BASE_RESULT_LISTENER_HPP */
