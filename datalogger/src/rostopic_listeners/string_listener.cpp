#include "datalogger/rostopic_listeners/string_listener.hpp"

namespace ros_listeners
{
    StringListener::StringListener(const std::string &topic_name, const std::string &topic_type, 
            const std::vector<std::string> &variable_names, double max_frequency,
            std::shared_ptr<loggers::DataLogger> data_logger)
        : ROSTopicListenerBase(topic_name, topic_type, variable_names, max_frequency, data_logger) { }

    StringListener::~StringListener() { }

    void StringListener::createSubscriber(std::shared_ptr<ros::NodeHandle> nh)
    {
        subscriber_.reset(new ros::Subscriber(
                    nh->subscribe<topic_tools::ShapeShifter>
                        (topic_name_, 1, boost::bind(&StringListener::log, this, _1))));
    }

    void StringListener::log(const topic_tools::ShapeShifter::ConstPtr &msg)
    {
        if (isTimeElapsed())
        {
            previous_msg_time_ = ros::Time::now();
            try
            {
                boost::shared_ptr<std_msgs::String> stringmsg = msg->instantiate<std_msgs::String>();

                Json::Value root;
                int var_num = 0;
                root["timestamp"] = getTimestamp();
                root["data"] = stringmsg->data;
                std::stringstream msg;
                msg << root;

                data_tunnel_interface_.tunnelData(readers::DataReader::formatVariableName(config::DataSourceNames::ROS, topic_name_),
                                                  msg.str());
            }
            catch (topic_tools::ShapeShifterException e)
            {
                std::cerr << "Shape shifter exception for message type std_msgs::String" << std::endl;
                exit(0);
            }
        }
    }
}
