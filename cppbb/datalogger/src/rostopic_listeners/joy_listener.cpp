#include "datalogger/rostopic_listeners/joy_listener.hpp"
#include <json/json.h>

namespace ros_listeners
{
    JoyListener::JoyListener(const std::string &topic_name, const std::string &topic_type, 
            const std::vector<std::string> &variable_names, double max_frequency,
            std::shared_ptr<loggers::DataLogger> data_logger)
        : ROSTopicListenerBase(topic_name, topic_type, variable_names, max_frequency, data_logger) { }

    JoyListener::~JoyListener() { }

    void JoyListener::createSubscriber(std::shared_ptr<ros::NodeHandle> nh)
    {
        subscriber_.reset(new ros::Subscriber(
                    nh->subscribe<topic_tools::ShapeShifter>
                        (topic_name_, 1, boost::bind(&JoyListener::log, this, _1))));
    }

    void JoyListener::log(const topic_tools::ShapeShifter::ConstPtr &msg)
    {
        if (isTimeElapsed())
        {
            previous_msg_time_ = ros::Time::now();
            try
            {
                boost::shared_ptr<sensor_msgs::Joy> joy = msg->instantiate<sensor_msgs::Joy>();

                std::vector<std::vector<float>> values;
                bool log = false;
                // only log non-zero joy values
                for (int i = 0; i < joy->buttons.size(); i++)
                {
                    if (joy->buttons[i] != 0)
                    {
                        log = true;
                        break;
                    }
                }
                if (!log)
                {
                    for (int i = 0; i < joy->axes.size(); i++)
                    {
                        if (joy->axes[i] != 0.0)
                        {
                            log = true;
                            break;
                        }
                    }
                }
                if (log)
                {
                    Json::Value root;
                    int var_num = 0;
                    root["timestamp"] = getTimestamp(joy->header.stamp);
                    Json::Value axes;
                    for (int i = 0; i < joy->axes.size(); i++)
                    {
                        axes.append(joy->axes[i]);
                    }
                    root[variable_names_[var_num++]] = axes;

                    Json::Value buttons;
                    for (int i = 0; i < joy->buttons.size(); i++)
                    {
                        buttons.append(joy->buttons[i]);
                    }
                    root[variable_names_[var_num++]] = buttons;

                    std::stringstream msg;
                    msg << root;

                    data_tunnel_interface_.tunnelData(readers::DataReader::formatVariableName(config::DataSourceNames::ROS, topic_name_),
                                                      msg.str());
                }
            }
            catch (topic_tools::ShapeShifterException e)
            {
                std::cerr << "Shape shifter exception for message type sensor_msgs::Joy" << std::endl;
                exit(0);
            }
        }
    }
}
