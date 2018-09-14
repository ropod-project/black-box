#include "datalogger/rostopic_listeners/rostopic_listener_factory.hpp"

namespace ros_listeners
{
    std::shared_ptr<ROSTopicListenerBase> ROSTopicListenerFactory::createListener(const std::string topic_name,
            const std::string topic_type, const std::vector<std::string> &variable_names,
            double max_frequency, std::shared_ptr<loggers::DataLogger> data_logger)
    {
        if (topic_type == "std_msgs/String")
        {
            std::shared_ptr<StringListener> listener(new StringListener(topic_name, topic_type,
                        variable_names, max_frequency, data_logger));
            return listener;
        }
        else if (topic_type == "nav_msgs/Odometry")
        {
            std::shared_ptr<OdomListener> listener(new OdomListener(topic_name, topic_type,
                        variable_names, max_frequency, data_logger));
            return listener;
        }
        else if (topic_type == "nav_msgs/Path")
        {
            std::shared_ptr<PathListener> listener(new PathListener(topic_name, topic_type,
                        variable_names, max_frequency, data_logger));
            return listener;
        }
        else if (topic_type == "move_base_msgs/MoveBaseActionResult")
        {
            std::shared_ptr<MoveBaseResultListener> listener(new MoveBaseResultListener(topic_name, topic_type,
                        variable_names, max_frequency, data_logger));
            return listener;
        }
        else if (topic_type == "geometry_msgs/Pose")
        {
            std::shared_ptr<PoseListener> listener(new PoseListener(topic_name, topic_type,
                        variable_names, max_frequency, data_logger));
            return listener;
        }
        else if (topic_type == "geometry_msgs/PoseStamped")
        {
            std::shared_ptr<PoseStampedListener> listener
                (new PoseStampedListener(topic_name, topic_type,
                         variable_names, max_frequency, data_logger));
            return listener;
        }
        else if (topic_type == "geometry_msgs/PoseWithCovarianceStamped")
        {
            std::shared_ptr<PoseWithCovarianceStampedListener> listener
                (new PoseWithCovarianceStampedListener(topic_name, topic_type,
                                                       variable_names, max_frequency, data_logger));
            return listener;
        }
        else if (topic_type == "geometry_msgs/Twist")
        {
            std::shared_ptr<TwistListener> listener(new TwistListener(topic_name, topic_type,
                        variable_names, max_frequency, data_logger));
            return listener;
        }
        else if (topic_type == "sensor_msgs/Joy")
        {
            std::shared_ptr<JoyListener> listener(new JoyListener(topic_name, topic_type,
                        variable_names, max_frequency, data_logger));
            return listener;
        }
        else if (topic_type == "sensor_msgs/Imu")
        {
            std::shared_ptr<IMUListener> listener(new IMUListener(topic_name, topic_type,
                        variable_names, max_frequency, data_logger));
            return listener;
        }
        else if (topic_type == "sensor_msgs/JointState")
        {
            std::shared_ptr<JointStateListener> listener(new JointStateListener(topic_name, topic_type,
                        variable_names, max_frequency, data_logger));
            return listener;
        }
        else if (topic_type == "sensor_msgs/LaserScan")
        {
            std::shared_ptr<LaserScanListener> listener(new LaserScanListener(topic_name, topic_type,
                        variable_names, max_frequency, data_logger));
            return listener;
        }
        else if (topic_type == "ropod_ros_msgs/ropod_door_detection")
        {
            std::shared_ptr<DoorListener> listener(new DoorListener(topic_name, topic_type,
                        variable_names, max_frequency, data_logger));
            return listener;
        }
        else if (topic_type == "ropod_ros_msgs/SmartWheelData")
        {
            std::shared_ptr<SmartWheelDataListener> listener(new SmartWheelDataListener(topic_name, topic_type,
                        variable_names, max_frequency, data_logger));
            return listener;
        }
    }
}
