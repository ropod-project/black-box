#include "datalogger/rostopic_listeners/pose_with_covariance_stamped_listener.hpp"
#include <json/json.h>

namespace ros_listeners
{
    PoseWithCovarianceStampedListener::PoseWithCovarianceStampedListener(const std::string &topic_name, const std::string &topic_type, 
            const std::vector<std::string> &variable_names, double max_frequency,
            std::shared_ptr<loggers::DataLogger> data_logger)
        : ROSTopicListenerBase(topic_name, topic_type, variable_names, max_frequency, data_logger) { }

    PoseWithCovarianceStampedListener::~PoseWithCovarianceStampedListener() { }

    void PoseWithCovarianceStampedListener::createSubscriber(std::shared_ptr<ros::NodeHandle> nh)
    {
        subscriber_.reset(new ros::Subscriber(
                    nh->subscribe<topic_tools::ShapeShifter>
                        (topic_name_, 1, boost::bind(&PoseWithCovarianceStampedListener::log, this, _1))));
    }

    void PoseWithCovarianceStampedListener::log(const topic_tools::ShapeShifter::ConstPtr &msg)
    {
        if (isTimeElapsed())
        {
            previous_msg_time_ = ros::Time::now();
            try
            {
                boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped> pose = 
                    msg->instantiate<geometry_msgs::PoseWithCovarianceStamped>();

                Json::Value root;
                int var_num = 0;
                root["timestamp"] = getTimestamp(pose->header.stamp);
                root[variable_names_[var_num++]] = pose->pose.pose.position.x;
                root[variable_names_[var_num++]] = pose->pose.pose.position.y;
                root[variable_names_[var_num++]] = pose->pose.pose.position.z;
                root[variable_names_[var_num++]] = pose->pose.pose.orientation.x;
                root[variable_names_[var_num++]] = pose->pose.pose.orientation.y;
                root[variable_names_[var_num++]] = pose->pose.pose.orientation.z;
                root[variable_names_[var_num++]] = pose->pose.pose.orientation.w;

                Json::Value cov;
                for (int i = 0; i < pose->pose.covariance.size(); i++)
                {
                    cov.append(pose->pose.covariance[i]);
                }
                root[variable_names_[var_num++]] = cov;

                std::stringstream msg;
                msg << root;

                data_tunnel_interface_.tunnelData(readers::DataReader::formatVariableName(config::DataSourceNames::ROS, topic_name_),
                                                  msg.str());
            }
            catch (topic_tools::ShapeShifterException e)
            {
                std::cerr << "Shape shifter exception for message type geometry_msgs::PoseWithCovarianceStamped" << std::endl;
                exit(0);
            }
        }
    }
}
