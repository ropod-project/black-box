#include "datalogger/rostopic_listeners/move_base_result_listener.hpp"
#include <json/json.h>

namespace ros_listeners
{
    MoveBaseResultListener::MoveBaseResultListener(const std::string &topic_name, const std::string &topic_type, 
            const std::vector<std::string> &variable_names, double max_frequency,
            std::shared_ptr<loggers::DataLogger> data_logger)
        : ROSTopicListenerBase(topic_name, topic_type, variable_names, max_frequency, data_logger) { }

    MoveBaseResultListener::~MoveBaseResultListener() { }

    void MoveBaseResultListener::createSubscriber(std::shared_ptr<ros::NodeHandle> nh)
    {
        subscriber_.reset(new ros::Subscriber(
                    nh->subscribe<topic_tools::ShapeShifter>
                        (topic_name_, 1, boost::bind(&MoveBaseResultListener::log, this, _1))));
    }

    void MoveBaseResultListener::log(const topic_tools::ShapeShifter::ConstPtr &msg)
    {
        if (isTimeElapsed())
        {
            previous_msg_time_ = ros::Time::now();
            try
            {
                boost::shared_ptr<move_base_msgs::MoveBaseActionResult> move_base_result
                    = msg->instantiate<move_base_msgs::MoveBaseActionResult>();

                Json::Value root;
                int var_num = 0;
                root["timestamp"] = getTimestamp(move_base_result->header.stamp);

                root[variable_names_[var_num++]] = move_base_result->status.status;
                root[variable_names_[var_num++]] = move_base_result->status.goal_id.id;
                root[variable_names_[var_num++]] = move_base_result->status.text;

                std::stringstream msg;
                msg << root;

                data_tunnel_interface_.tunnelData(readers::DataReader::formatVariableName(config::DataSourceNames::ROS, topic_name_),
                                                  msg.str());
            }
            catch (topic_tools::ShapeShifterException e)
            {
                std::cerr << "Shape shifter exception for message type move_base_msgs::MoveBaseActionResult" << std::endl;
                exit(0);
            }
        }
    }
}
