#include "datalogger/rostopic_listeners/path_listener.hpp"
#include <json/json.h>

namespace ros_listeners
{
    PathListener::PathListener(const std::string &topic_name, const std::string &topic_type,
            const std::vector<std::string> &variable_names, double max_frequency,
            std::shared_ptr<loggers::DataLogger> data_logger)
        : ROSTopicListenerBase(topic_name, topic_type, variable_names, max_frequency, data_logger) { }

    PathListener::~PathListener() { }

    void PathListener::createSubscriber(std::shared_ptr<ros::NodeHandle> nh)
    {
        subscriber_.reset(new ros::Subscriber(
                    nh->subscribe<topic_tools::ShapeShifter>
                        (topic_name_, 1, boost::bind(&PathListener::log, this, _1))));
    }

    void PathListener::log(const topic_tools::ShapeShifter::ConstPtr &msg)
    {
        if (isTimeElapsed())
        {
            previous_msg_time_ = ros::Time::now();
            try
            {
                boost::shared_ptr<nav_msgs::Path> path = msg->instantiate<nav_msgs::Path>();

                Json::Value root;
                int var_num = 0;
                root["timestamp"] = getTimestamp(path->header.stamp);
                Json::Value poses;
                for (int i = 0; i < path->poses.size(); i++)
                {
                    Json::Value pose;
                    pose["position_x"] = path->poses[i].pose.position.x;
                    pose["position_y"] = path->poses[i].pose.position.y;
                    pose["position_z"] = path->poses[i].pose.position.z;
                    pose["orientation_x"] = path->poses[i].pose.orientation.x;
                    pose["orientation_y"] = path->poses[i].pose.orientation.y;
                    pose["orientation_z"] = path->poses[i].pose.orientation.z;
                    pose["orientation_w"] = path->poses[i].pose.orientation.w;
                    poses.append(pose);
                }
                root[variable_names_[var_num++]] = poses;

                std::stringstream msg;
                msg << root;

                data_tunnel_interface_.tunnelData(readers::DataReader::formatVariableName(config::DataSourceNames::ROS, topic_name_),
                                                  msg.str());
            }
            catch (topic_tools::ShapeShifterException e)
            {
                std::cerr << "Shape shifter exception for message type nav_msgs::Path" << std::endl;
                exit(0);
            }
        }
    }
}
