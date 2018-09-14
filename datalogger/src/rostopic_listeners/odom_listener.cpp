#include "datalogger/rostopic_listeners/odom_listener.hpp"
#include <json/json.h>

namespace ros_listeners
{
    OdomListener::OdomListener(const std::string &topic_name, const std::string &topic_type,
            const std::vector<std::string> &variable_names, double max_frequency,
            std::shared_ptr<loggers::DataLogger> data_logger)
        : ROSTopicListenerBase(topic_name, topic_type, variable_names, max_frequency, data_logger) { }

    OdomListener::~OdomListener() { }

    void OdomListener::createSubscriber(std::shared_ptr<ros::NodeHandle> nh)
    {
        subscriber_.reset(new ros::Subscriber(
                    nh->subscribe<topic_tools::ShapeShifter>
                        (topic_name_, 1, boost::bind(&OdomListener::log, this, _1))));
    }

    void OdomListener::log(const topic_tools::ShapeShifter::ConstPtr &msg)
    {
        if (isTimeElapsed())
        {
            previous_msg_time_ = ros::Time::now();
            try
            {
                boost::shared_ptr<nav_msgs::Odometry> odom = msg->instantiate<nav_msgs::Odometry>();

                Json::Value root;
                int var_num = 0;
                root["timestamp"] = getTimestamp(odom->header.stamp);
                root[variable_names_[var_num++]] = odom->pose.pose.position.x;
                root[variable_names_[var_num++]] = odom->pose.pose.position.y;
                root[variable_names_[var_num++]] = odom->pose.pose.position.z;
                root[variable_names_[var_num++]] = odom->pose.pose.orientation.x;
                root[variable_names_[var_num++]] = odom->pose.pose.orientation.y;
                root[variable_names_[var_num++]] = odom->pose.pose.orientation.z;
                root[variable_names_[var_num++]] = odom->pose.pose.orientation.w;
                root[variable_names_[var_num++]] = odom->twist.twist.linear.x;
                root[variable_names_[var_num++]] = odom->twist.twist.linear.y;
                root[variable_names_[var_num++]] = odom->twist.twist.linear.z;
                root[variable_names_[var_num++]] = odom->twist.twist.angular.x;
                root[variable_names_[var_num++]] = odom->twist.twist.angular.y;
                root[variable_names_[var_num++]] = odom->twist.twist.angular.z;

                std::stringstream msg;
                msg << root;

                data_tunnel_interface_.tunnelData(readers::DataReader::formatVariableName(config::DataSourceNames::ROS, topic_name_),
                                                  msg.str());
            }
            catch (topic_tools::ShapeShifterException e)
            {
                std::cerr << "Shape shifter exception for message type nav_msgs::Odometry" << std::endl;
                exit(0);
            }
        }
    }
}
