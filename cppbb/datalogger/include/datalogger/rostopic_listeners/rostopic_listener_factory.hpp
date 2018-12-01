#ifndef ROSTOPIC_LISTENER_FACTORY_H
#define ROSTOPIC_LISTENER_FACTORY_H

#include "datalogger/data_loggers/data_logger.hpp"
#include "datalogger/rostopic_listeners/rostopic_listener_base.hpp"
#include "datalogger/rostopic_listeners/string_listener.hpp"
#include "datalogger/rostopic_listeners/odom_listener.hpp"
#include "datalogger/rostopic_listeners/path_listener.hpp"
#include "datalogger/rostopic_listeners/pose_listener.hpp"
#include "datalogger/rostopic_listeners/pose_stamped_listener.hpp"
#include "datalogger/rostopic_listeners/wrench_stamped_listener.hpp"
#include "datalogger/rostopic_listeners/pose_with_covariance_stamped_listener.hpp"
#include "datalogger/rostopic_listeners/imu_listener.hpp"
#include "datalogger/rostopic_listeners/joy_listener.hpp"
#include "datalogger/rostopic_listeners/joint_state_listener.hpp"
#include "datalogger/rostopic_listeners/twist_listener.hpp"
#include "datalogger/rostopic_listeners/laserscan_listener.hpp"
#include "datalogger/rostopic_listeners/move_base_result_listener.hpp"
#include "datalogger/rostopic_listeners/door_listener.hpp"
#include "datalogger/rostopic_listeners/smart_wheel_data_listener.hpp"

namespace ros_listeners
{
    class ROSTopicListenerFactory
    {
    public:
        static std::shared_ptr<ROSTopicListenerBase> createListener(const std::string topic_name,
                const std::string topic_type, const std::vector<std::string> &variable_names,
                double max_frequency, std::shared_ptr<loggers::DataLogger> data_logger);
    };
}

#endif
