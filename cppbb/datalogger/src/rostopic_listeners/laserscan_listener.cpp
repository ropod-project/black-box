#include "datalogger/rostopic_listeners/laserscan_listener.hpp"

namespace ros_listeners
{
    LaserScanListener::LaserScanListener(const std::string &topic_name, const std::string &topic_type, 
            const std::vector<std::string> &variable_names, double max_frequency,
            std::shared_ptr<loggers::DataLogger> data_logger)
        : ROSTopicListenerBase(topic_name, topic_type, variable_names, max_frequency, data_logger) { }

    LaserScanListener::~LaserScanListener() { }

    void LaserScanListener::createSubscriber(std::shared_ptr<ros::NodeHandle> nh)
    {
        subscriber_.reset(new ros::Subscriber(
                    nh->subscribe<topic_tools::ShapeShifter>
                        (topic_name_, 1, boost::bind(&LaserScanListener::log, this, _1))));
    }

    void LaserScanListener::log(const topic_tools::ShapeShifter::ConstPtr &msg)
    {
        if (isTimeElapsed())
        {
            previous_msg_time_ = ros::Time::now();
            try
            {
                boost::shared_ptr<sensor_msgs::LaserScan> laser_scan = msg->instantiate<sensor_msgs::LaserScan>();

                Json::Value root;
                int var_num = 0;
                root["timestamp"] = getTimestamp(laser_scan->header.stamp);
                root[variable_names_[var_num++]] = laser_scan->header.frame_id;
                root[variable_names_[var_num++]] = laser_scan->angle_min;
                root[variable_names_[var_num++]] = laser_scan->angle_max;
                root[variable_names_[var_num++]] = laser_scan->angle_increment;
                root[variable_names_[var_num++]] = laser_scan->time_increment;
                root[variable_names_[var_num++]] = laser_scan->scan_time;
                root[variable_names_[var_num++]] = laser_scan->range_min;
                root[variable_names_[var_num++]] = laser_scan->range_max;
                Json::Value ranges;
                for (int i = 0; i < laser_scan->ranges.size(); i++)
                {
                    if (std::isinf(laser_scan->ranges[i]))
                    {
                        ranges.append("inf");
                    }
                    else if (std::isnan(laser_scan->ranges[i]))
                    {
                        ranges.append("nan");
                    }
                    else
                    {
                        ranges.append(laser_scan->ranges[i]);
                    }

                }
                root[variable_names_[var_num++]] = ranges;

                std::stringstream msg;
                msg << root;

                data_tunnel_interface_.tunnelData(readers::DataReader::formatVariableName(config::DataSourceNames::ROS, topic_name_),
                                                  msg.str());
            }
            catch (topic_tools::ShapeShifterException e)
            {
                std::cerr << "Shape shifter exception for message type sensor_msgs::LaserScan" << std::endl;
                exit(0);
            }
        }
    }
}
