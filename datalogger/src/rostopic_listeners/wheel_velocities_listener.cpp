#include "datalogger/rostopic_listeners/wheel_velocities_listener.hpp"

namespace ros_listeners
{
    WheelVelocitiesListener::WheelVelocitiesListener(const std::string &topic_name, const std::string &topic_type, 
            const std::vector<std::string> &variable_names, double max_frequency,
            std::shared_ptr<loggers::DataLogger> data_logger)
        : ROSTopicListenerBase(topic_name, topic_type, variable_names, max_frequency, data_logger) { }

    WheelVelocitiesListener::~WheelVelocitiesListener() { }

    void WheelVelocitiesListener::createSubscriber(std::shared_ptr<ros::NodeHandle> nh)
    {
        subscriber_.reset(new ros::Subscriber(
                    nh->subscribe<topic_tools::ShapeShifter>
                        (topic_name_, 1, boost::bind(&WheelVelocitiesListener::log, this, _1))));
    }

    void WheelVelocitiesListener::log(const topic_tools::ShapeShifter::ConstPtr &msg)
    {
        if (isTimeElapsed())
        {
            previous_msg_time_ = ros::Time::now();
            try
            {
                boost::shared_ptr<youbot_driver_ros_interface::WheelVelocities> wheel_velocities =
                    msg->instantiate<youbot_driver_ros_interface::WheelVelocities>();

                Json::Value root;
                int var_num = 0;
                root["timestamp"] = getTimestamp();
                root[variable_names_[var_num++]] = wheel_velocities->wheel1_velocity;
                root[variable_names_[var_num++]] = wheel_velocities->wheel2_velocity;
                root[variable_names_[var_num++]] = wheel_velocities->wheel3_velocity;
                root[variable_names_[var_num++]] = wheel_velocities->wheel4_velocity;
                root[variable_names_[var_num++]] = wheel_velocities->wheel1_expected_velocity;
                root[variable_names_[var_num++]] = wheel_velocities->wheel2_expected_velocity;
                root[variable_names_[var_num++]] = wheel_velocities->wheel3_expected_velocity;
                root[variable_names_[var_num++]] = wheel_velocities->wheel4_expected_velocity;

                std::stringstream msg;
                msg << root;

                data_tunnel_interface_.tunnelData(readers::DataReader::formatVariableName(config::DataSourceNames::ROS, topic_name_),
                                                  msg.str());
            }
            catch (topic_tools::ShapeShifterException e)
            {
                std::cerr << "Shape shifter exception for message type youbot_driver_ros_interface::WheelVelocities" << std::endl;
                exit(0);
            }
        }
    }
}
