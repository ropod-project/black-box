#include "datalogger/rostopic_listeners/imu_listener.hpp"

namespace ros_listeners
{
    IMUListener::IMUListener(const std::string &topic_name, const std::string &topic_type, 
            const std::vector<std::string> &variable_names, double max_frequency,
            std::shared_ptr<loggers::DataLogger> data_logger)
        : ROSTopicListenerBase(topic_name, topic_type, variable_names, max_frequency, data_logger) { }

    IMUListener::~IMUListener() { }

    void IMUListener::createSubscriber(std::shared_ptr<ros::NodeHandle> nh)
    {
        subscriber_.reset(new ros::Subscriber(
                    nh->subscribe<topic_tools::ShapeShifter>
                        (topic_name_, 1, boost::bind(&IMUListener::log, this, _1))));
    }

    void IMUListener::log(const topic_tools::ShapeShifter::ConstPtr &msg)
    {
        if (isTimeElapsed())
        {
            previous_msg_time_ = ros::Time::now();
            try
            {
                boost::shared_ptr<sensor_msgs::Imu> imu = msg->instantiate<sensor_msgs::Imu>();

                Json::Value root;
                int var_num = 0;
                root["timestamp"] = getTimestamp(imu->header.stamp);
                root[variable_names_[var_num++]] = imu->orientation.x;
                root[variable_names_[var_num++]] = imu->orientation.y;
                root[variable_names_[var_num++]] = imu->orientation.z;
                root[variable_names_[var_num++]] = imu->orientation.w;
                root[variable_names_[var_num++]] = imu->angular_velocity.x;
                root[variable_names_[var_num++]] = imu->angular_velocity.y;
                root[variable_names_[var_num++]] = imu->angular_velocity.z;
                root[variable_names_[var_num++]] = imu->linear_acceleration.x;
                root[variable_names_[var_num++]] = imu->linear_acceleration.y;
                root[variable_names_[var_num++]] = imu->linear_acceleration.z;

                std::stringstream msg;
                msg << root;

                data_tunnel_interface_.tunnelData(readers::DataReader::formatVariableName(config::DataSourceNames::ROS, topic_name_),
                                                  msg.str());
            }
            catch (topic_tools::ShapeShifterException e)
            {
                std::cerr << "Shape shifter exception for message type sensor_msgs::Imu" << std::endl;
                exit(0);
            }
        }
    }
}
