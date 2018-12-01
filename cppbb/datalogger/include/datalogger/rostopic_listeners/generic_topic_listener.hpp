#ifndef GENERIC_TOPIC_LISTENER_H
#define GENERIC_TOPIC_LISTENER_H

#include <ros/ros.h>

#include "datalogger/rostopic_listeners/rostopic_listener_base.hpp"
#include <ros_type_introspection/ros_introspection.hpp>

namespace ros_listeners
{
    class GenericTopicListener : public ROSTopicListenerBase
    {
    public:
        GenericTopicListener(const std::string &topic_name, const std::string &topic_type, std::shared_ptr<loggers::DataLogger> data_logger);
        virtual ~GenericTopicListener();

        virtual void createSubscriber(std::shared_ptr<ros::NodeHandle> nh);
        virtual void log(const topic_tools::ShapeShifter::ConstPtr &msg);

    protected:
        RosIntrospection::ROSTypeList registered_type_;
        bool is_registered_type_set_;
    };
}

#endif /* GENERIC_TOPIC_LISTENER_H */
