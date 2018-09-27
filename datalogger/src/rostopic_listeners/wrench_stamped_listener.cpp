#include "datalogger/rostopic_listeners/wrench_stamped_listener.hpp"
#include <json/json.h>

namespace ros_listeners
{
    WrenchStampedListener::WrenchStampedListener(const std::string &topic_name, const std::string &topic_type, 
            const std::vector<std::string> &variable_names, double max_frequency,
            std::shared_ptr<loggers::DataLogger> data_logger)
        : ROSTopicListenerBase(topic_name, topic_type, variable_names, max_frequency, data_logger) { }

    WrenchStampedListener::~WrenchStampedListener() { }

    void WrenchStampedListener::createSubscriber(std::shared_ptr<ros::NodeHandle> nh)
    {
        subscriber_.reset(new ros::Subscriber(
                    nh->subscribe<topic_tools::ShapeShifter>
                        (topic_name_, 1, boost::bind(&WrenchStampedListener::log, this, _1))));
    }

    void WrenchStampedListener::log(const topic_tools::ShapeShifter::ConstPtr &msg)
    {
        if (isTimeElapsed())
        {
            previous_msg_time_ = ros::Time::now();
            try
            {
                boost::shared_ptr<geometry_msgs::WrenchStamped> wrench = msg->instantiate<geometry_msgs::WrenchStamped>();

                Json::Value root;
                int var_num = 0;
                root["timestamp"] = getTimestamp(wrench->header.stamp);
                root[variable_names_[var_num++]] = wrench->wrench.force.x;
                root[variable_names_[var_num++]] = wrench->wrench.force.y;
                root[variable_names_[var_num++]] = wrench->wrench.force.z;
                root[variable_names_[var_num++]] = wrench->wrench.torque.x;
                root[variable_names_[var_num++]] = wrench->wrench.torque.y;
                root[variable_names_[var_num++]] = wrench->wrench.torque.z;
                root[variable_names_[var_num++]] = wrench->header.frame_id;

                std::stringstream msg;
                msg << root;

                data_tunnel_interface_.tunnelData(readers::DataReader::formatVariableName(config::DataSourceNames::ROS, topic_name_),
                                                  msg.str());
            }
            catch (topic_tools::ShapeShifterException e)
            {
                std::cerr << "Shape shifter exception for message type geometry_msgs::WrenchStamped" << std::endl;
                exit(0);
            }
        }
    }
}
