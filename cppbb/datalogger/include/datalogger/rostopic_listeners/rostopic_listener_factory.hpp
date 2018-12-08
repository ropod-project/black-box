#ifndef ROSTOPIC_LISTENER_FACTORY_H
#define ROSTOPIC_LISTENER_FACTORY_H

#include "datalogger/data_loggers/data_logger.hpp"
#include "datalogger/rostopic_listeners/rostopic_listener_base.hpp"
#include "datalogger/rostopic_listeners/generic_topic_listener.hpp"

namespace ros_listeners
{
    class ROSTopicListenerFactory
    {
    public:
        static std::shared_ptr<ROSTopicListenerBase> createListener(const std::string topic_name,
                const std::string topic_type, const std::vector<std::string> &variable_names,
                double max_frequency, std::shared_ptr<loggers::DataLogger> data_logger);
    };
}

#endif
