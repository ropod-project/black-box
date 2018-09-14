#include "datalogger/rostopic_listeners/joint_state_listener.hpp"
#include <json/json.h>

namespace ros_listeners
{
    JointStateListener::JointStateListener(const std::string &topic_name, const std::string &topic_type, 
            const std::vector<std::string> &variable_names, double max_frequency,
            std::shared_ptr<loggers::DataLogger> data_logger)
        : ROSTopicListenerBase(topic_name, topic_type, variable_names, max_frequency, data_logger) { }

    JointStateListener::~JointStateListener() { }

    void JointStateListener::createSubscriber(std::shared_ptr<ros::NodeHandle> nh)
    {
        subscriber_.reset(new ros::Subscriber(
                    nh->subscribe<topic_tools::ShapeShifter>
                        (topic_name_, 1, boost::bind(&JointStateListener::log, this, _1))));
    }

    void JointStateListener::log(const topic_tools::ShapeShifter::ConstPtr &msg)
    {
        if (isTimeElapsed())
        {
            previous_msg_time_ = ros::Time::now();
            try
            {
                boost::shared_ptr<sensor_msgs::JointState> js = msg->instantiate<sensor_msgs::JointState>();

                Json::Value root;
                int var_num = 0;
                root["timestamp"] = getTimestamp(js->header.stamp);
                Json::Value names;
                Json::Value positions;
                Json::Value velocities;
                Json::Value efforts;
                for (int i = 0; i < js->name.size(); i++)
                {
                    names.append(js->name[i]);
                }
                for (int i = 0; i < js->position.size(); i++)
                {
                    positions.append(js->position[i]);
                }
                for (int i = 0; i < js->velocity.size(); i++)
                {
                    velocities.append(js->velocity[i]);
                }
                for (int i = 0; i < js->effort.size(); i++)
                {
                    efforts.append(js->effort[i]);
                }
                root[variable_names_[var_num++]] = names;
                root[variable_names_[var_num++]] = positions;
                root[variable_names_[var_num++]] = velocities;
                root[variable_names_[var_num++]] = efforts;
                root[variable_names_[var_num++]] = js->header.frame_id;

                std::stringstream msg;
                msg << root;

                data_tunnel_interface_.tunnelData(readers::DataReader::formatVariableName(config::DataSourceNames::ROS, topic_name_),
                                                  msg.str());
            }
            catch (topic_tools::ShapeShifterException e)
            {
                std::cerr << "Shape shifter exception for message type sensor_msgs::JointState" << std::endl;
                exit(0);
            }
        }
    }
}
