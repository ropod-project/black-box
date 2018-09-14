#include "datalogger/rostopic_listeners/pose_listener.hpp"
#include <json/json.h>

namespace ros_listeners
{
    PoseListener::PoseListener(const std::string &topic_name, const std::string &topic_type, 
            const std::vector<std::string> &variable_names, double max_frequency,
            std::shared_ptr<loggers::DataLogger> data_logger)
        : ROSTopicListenerBase(topic_name, topic_type, variable_names, max_frequency, data_logger) { }

    PoseListener::~PoseListener() { }

    void PoseListener::createSubscriber(std::shared_ptr<ros::NodeHandle> nh)
    {
        subscriber_.reset(new ros::Subscriber(
                    nh->subscribe<topic_tools::ShapeShifter>
                        (topic_name_, 1, boost::bind(&PoseListener::log, this, _1))));
    }

    void PoseListener::log(const topic_tools::ShapeShifter::ConstPtr &msg)
    {
        if (isTimeElapsed())
        {
            previous_msg_time_ = ros::Time::now();
            try
            {
                boost::shared_ptr<geometry_msgs::Pose> pose = msg->instantiate<geometry_msgs::Pose>();

                Json::Value root;
                int var_num = 0;
                root["timestamp"] = getTimestamp();
                root[variable_names_[var_num++]] = pose->position.x;
                root[variable_names_[var_num++]] = pose->position.y;
                root[variable_names_[var_num++]] = pose->position.z;
                root[variable_names_[var_num++]] = pose->orientation.x;
                root[variable_names_[var_num++]] = pose->orientation.y;
                root[variable_names_[var_num++]] = pose->orientation.z;
                root[variable_names_[var_num++]] = pose->orientation.w;

                std::stringstream msg;
                msg << root;

                data_tunnel_interface_.tunnelData(readers::DataReader::formatVariableName(config::DataSourceNames::ROS, topic_name_),
                                                  msg.str());
            }
            catch (topic_tools::ShapeShifterException e)
            {
                std::cerr << "Shape shifter exception for message type geometry_msgs::Pose" << std::endl;
                exit(0);
            }
        }
    }
}
