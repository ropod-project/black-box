#ifndef IMU_LISTENER_H
#define IMU_LISTENER_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "datalogger/rostopic_listeners/rostopic_listener_base.hpp"

namespace ros_listeners
{
    class IMUListener : public ROSTopicListenerBase
    {
    public:
        IMUListener(const std::string &topic_name, const std::string &topic_type, 
                const std::vector<std::string> &variable_names, double max_frequency,
                std::shared_ptr<loggers::DataLogger> data_logger);
        virtual ~IMUListener();

        virtual void createSubscriber(std::shared_ptr<ros::NodeHandle> nh);
        virtual void log(const topic_tools::ShapeShifter::ConstPtr &msg);
    };
}

#endif /* IMU_LISTENER_H */
