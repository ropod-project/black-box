#include "datalogger/rostopic_listeners/twist_listener.hpp"

namespace ros_listeners
{
    TwistListener::TwistListener(const std::string &topic_name, const std::string &topic_type, 
            const std::vector<std::string> &variable_names, double max_frequency,
            std::shared_ptr<loggers::DataLogger> data_logger)
        : ROSTopicListenerBase(topic_name, topic_type, variable_names, max_frequency, data_logger) { }

    TwistListener::~TwistListener() { }

    void TwistListener::createSubscriber(std::shared_ptr<ros::NodeHandle> nh)
    {
        subscriber_.reset(new ros::Subscriber(
                    nh->subscribe<topic_tools::ShapeShifter>
                        (topic_name_, 1, boost::bind(&TwistListener::log, this, _1))));
    }

    void TwistListener::log(const topic_tools::ShapeShifter::ConstPtr &msg)
    {
        if (isTimeElapsed())
        {
            previous_msg_time_ = ros::Time::now();
            try
            {
                boost::shared_ptr<geometry_msgs::Twist> twist = msg->instantiate<geometry_msgs::Twist>();

                Json::Value root;
                int var_num = 0;
                root["timestamp"] = getTimestamp();
                root[variable_names_[var_num++]] = twist->linear.x;
                root[variable_names_[var_num++]] = twist->linear.y;
                root[variable_names_[var_num++]] = twist->linear.z;
                root[variable_names_[var_num++]] = twist->angular.x;
                root[variable_names_[var_num++]] = twist->angular.y;
                root[variable_names_[var_num++]] = twist->angular.z;

                std::stringstream msg;
                msg << root;

                data_tunnel_interface_.tunnelData(readers::DataReader::formatVariableName(config::DataSourceNames::ROS, topic_name_),
                                                  msg.str());
            }
            catch (topic_tools::ShapeShifterException e)
            {
                std::cerr << "Shape shifter exception for message type geometry_msgs::Twist" << std::endl;
                exit(0);
            }
        }
    }
}
