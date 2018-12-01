#include "datalogger/rostopic_listeners/door_listener.hpp"

namespace ros_listeners
{
    DoorListener::DoorListener(const std::string &topic_name, const std::string &topic_type,
            const std::vector<std::string> &variable_names, double max_frequency,
            std::shared_ptr<loggers::DataLogger> data_logger)
        : ROSTopicListenerBase(topic_name, topic_type, variable_names, max_frequency, data_logger) { }

    DoorListener::~DoorListener() { }

    void DoorListener::createSubscriber(std::shared_ptr<ros::NodeHandle> nh)
    {
        subscriber_.reset(new ros::Subscriber(
                    nh->subscribe<topic_tools::ShapeShifter>
                        (topic_name_, 1, boost::bind(&DoorListener::log, this, _1))));
    }

    void DoorListener::log(const topic_tools::ShapeShifter::ConstPtr &msg)
    {
        if (isTimeElapsed())
        {
            previous_msg_time_ = ros::Time::now();
            try
            {
                boost::shared_ptr<ropod_ros_msgs::DoorDetection> door
                    = msg->instantiate<ropod_ros_msgs::DoorDetection>();

                Json::Value root;
                std::vector<float> values;
                int var_num = 0;
                root["timestamp"] = getTimestamp(door->header.stamp);
                root[variable_names_[var_num++]] = door->open;
                root[variable_names_[var_num++]] = door->closed;
                root[variable_names_[var_num++]] = door->undetectable;
                std::stringstream msg;
                msg << root;

                data_tunnel_interface_.tunnelData(readers::DataReader::formatVariableName(config::DataSourceNames::ROS, topic_name_),
                                                  msg.str());
            }
            catch (topic_tools::ShapeShifterException e)
            {
                std::cerr << "Shape shifter exception for message type ropod_ros_msgs::DoorDetection" << std::endl;
                exit(0);
            }
        }
    }
}
