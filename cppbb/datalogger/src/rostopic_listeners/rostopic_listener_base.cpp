#include "datalogger/rostopic_listeners/rostopic_listener_base.hpp"
#include <chrono>

namespace ros_listeners
{
    ROSTopicListenerBase::ROSTopicListenerBase(const std::string &topic_name, const std::string &topic_type,
            double max_frequency, std::shared_ptr<loggers::DataLogger> data_logger)
        : topic_name_(topic_name), topic_type_(topic_type),
          max_frequency_(max_frequency), data_tunnel_interface_(data_logger)
        {
            minimum_period_ = ros::Duration(1.0/max_frequency_);
            previous_msg_time_ = ros::Time::now();
        }

    ROSTopicListenerBase::~ROSTopicListenerBase()
    {
        this->subscriber_->shutdown();
    }

    void ROSTopicListenerBase::shutdownSubscriber()
    {
        this->subscriber_->shutdown();
    }

    void ROSTopicListenerBase::resetSubscriber()
    {
        this->subscriber_.reset();
    }

    double ROSTopicListenerBase::getTimestamp()
    {
        auto now = std::chrono::system_clock::now();
        return std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() / 1000.0;
    }

    double ROSTopicListenerBase::getTimestamp(ros::Time time)
    {
        return (time.toSec());
    }

    bool ROSTopicListenerBase::isTimeElapsed()
    {
        ros::Time now = ros::Time::now();
        if (now - previous_msg_time_ > minimum_period_)
        {
            return true;
        }
        return false;
    }
}
