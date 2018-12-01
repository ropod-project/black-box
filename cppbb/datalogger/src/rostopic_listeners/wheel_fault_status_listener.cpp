#include "datalogger/rostopic_listeners/wheel_fault_status_listener.hpp"

namespace ros_listeners
{
    WheelFaultStatusListener::WheelFaultStatusListener(const std::string &topic_name, const std::string &topic_type,
            const std::vector<std::string> &variable_names, double max_frequency,
            std::shared_ptr<loggers::DataLogger> data_logger)
        : ROSTopicListenerBase(topic_name, topic_type, variable_names, max_frequency, data_logger) { }

    WheelFaultStatusListener::~WheelFaultStatusListener() { }

    void WheelFaultStatusListener::createSubscriber(std::shared_ptr<ros::NodeHandle> nh)
    {
        subscriber_.reset(new ros::Subscriber(
                    nh->subscribe<topic_tools::ShapeShifter>
                        (topic_name_, 1, boost::bind(&WheelFaultStatusListener::log, this, _1))));
    }

    void WheelFaultStatusListener::log(const topic_tools::ShapeShifter::ConstPtr &msg)
    {
        if (isTimeElapsed())
        {
            previous_msg_time_ = ros::Time::now();
            try
            {
                boost::shared_ptr<youbot_driver_ros_interface::WheelFaultStatus> fault_status =
                    msg->instantiate<youbot_driver_ros_interface::WheelFaultStatus>();

                Json::Value root;
                int var_num = 0;
                root["timestamp"] = getTimestamp();
                root[variable_names_[var_num++]] = fault_status->wheel1_status;
                root[variable_names_[var_num++]] = fault_status->wheel2_status;
                root[variable_names_[var_num++]] = fault_status->wheel3_status;
                root[variable_names_[var_num++]] = fault_status->wheel4_status;

                std::stringstream msg;
                msg << root;

                data_tunnel_interface_.tunnelData(readers::DataReader::formatVariableName(config::DataSourceNames::ROS, topic_name_),
                                                  msg.str());
            }
            catch (topic_tools::ShapeShifterException e)
            {
                std::cerr << "Shape shifter exception for message type youbot_driver_ros_interface::WheelFaultStatus" << std::endl;
                exit(0);
            }
        }
    }
}
