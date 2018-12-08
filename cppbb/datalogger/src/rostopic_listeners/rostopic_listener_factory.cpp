#include "datalogger/rostopic_listeners/rostopic_listener_factory.hpp"

namespace ros_listeners
{
    std::shared_ptr<ROSTopicListenerBase> ROSTopicListenerFactory::createListener(const std::string topic_name,
            const std::string topic_type, const std::vector<std::string> &variable_names,
            double max_frequency, std::shared_ptr<loggers::DataLogger> data_logger)
    {
        std::shared_ptr<GenericTopicListener> listener(new GenericTopicListener(topic_name, topic_type,
                    max_frequency, data_logger));
        return listener;
    }
}
