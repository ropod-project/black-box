#include "datalogger/rostopic_listeners/smart_wheel_data_listener.hpp"
#include <json/json.h>

namespace ros_listeners
{
    SmartWheelDataListener::SmartWheelDataListener(const std::string &topic_name, const std::string &topic_type,
            const std::vector<std::string> &variable_names, double max_frequency,
            std::shared_ptr<loggers::DataLogger> data_logger)
        : ROSTopicListenerBase(topic_name, topic_type, variable_names, max_frequency, data_logger) { }

    SmartWheelDataListener::~SmartWheelDataListener() { }

    void SmartWheelDataListener::createSubscriber(std::shared_ptr<ros::NodeHandle> nh)
    {
        subscriber_.reset(new ros::Subscriber(
                    nh->subscribe<topic_tools::ShapeShifter>
                        (topic_name_, 1, boost::bind(&SmartWheelDataListener::log, this, _1))));
    }

    void SmartWheelDataListener::log(const topic_tools::ShapeShifter::ConstPtr &msg)
    {
        if (isTimeElapsed())
        {
            previous_msg_time_ = ros::Time::now();
            try
            {
                boost::shared_ptr<ropod_ros_msgs::SmartWheelData> swdata = msg->instantiate<ropod_ros_msgs::SmartWheelData>();

                Json::Value root;
                int var_num = 0;
                root["timestamp"] = getTimestamp(swdata->header.stamp);
                for (int i = 0; i < swdata->commands.size(); i++)
                {
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/command1"] = swdata->commands[i].command1;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/command2"] = swdata->commands[i].command2;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/setpoint1"] = swdata->commands[i].setpoint1;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/setpoint2"] = swdata->commands[i].setpoint2;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/limit1_p"] = swdata->commands[i].limit1_p;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/limit1_n"] = swdata->commands[i].limit1_n;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/limit2_p"] = swdata->commands[i].limit2_p;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/limit2_n"] = swdata->commands[i].limit2_n;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/timestamp"] = swdata->commands[i].timestamp;
                }
                var_num += 1;

                for (int i = 0; i < swdata->sensors.size(); i++)
                {
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/status1"] = swdata->sensors[i].status1;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/status2"] = swdata->sensors[i].status2;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/sensor_ts"] = swdata->sensors[i].sensor_ts;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/setpoint_ts"] = swdata->sensors[i].setpoint_ts;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/encoder_1"] = swdata->sensors[i].encoder_1;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/velocity_1"] = swdata->sensors[i].velocity_1;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/current_1_d"] = swdata->sensors[i].current_1_d;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/current_1_q"] = swdata->sensors[i].current_1_q;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/current_1_u"] = swdata->sensors[i].current_1_u;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/current_1_v"] = swdata->sensors[i].current_1_v;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/current_1_w"] = swdata->sensors[i].current_1_w;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/voltage_1"] = swdata->sensors[i].voltage_1;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/voltage_1_u"] = swdata->sensors[i].voltage_1_u;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/voltage_1_v"] = swdata->sensors[i].voltage_1_v;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/voltage_1_w"] = swdata->sensors[i].voltage_1_w;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/temperature_1"] = swdata->sensors[i].temperature_1;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/encoder_2"] = swdata->sensors[i].encoder_2;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/velocity_2"] = swdata->sensors[i].velocity_2;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/current_2_d"] = swdata->sensors[i].current_2_d;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/current_2_q"] = swdata->sensors[i].current_2_q;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/current_2_u"] = swdata->sensors[i].current_2_u;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/current_2_v"] = swdata->sensors[i].current_2_v;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/current_2_w"] = swdata->sensors[i].current_2_w;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/voltage_2"] = swdata->sensors[i].voltage_2;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/voltage_2_u"] = swdata->sensors[i].voltage_2_u;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/voltage_2_v"] = swdata->sensors[i].voltage_2_v;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/voltage_2_w"] = swdata->sensors[i].voltage_2_w;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/temperature_2"] = swdata->sensors[i].temperature_2;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/encoder_pivot"] = swdata->sensors[i].encoder_pivot;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/velocity_pivot"] = swdata->sensors[i].velocity_pivot;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/voltage_bus"] = swdata->sensors[i].voltage_bus;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/imu_ts"] = swdata->sensors[i].imu_ts;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/accel_x"] = swdata->sensors[i].accel_x;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/accel_y"] = swdata->sensors[i].accel_y;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/accel_z"] = swdata->sensors[i].accel_z;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/gyro_x"] = swdata->sensors[i].gyro_x;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/gyro_y"] = swdata->sensors[i].gyro_y;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/gyro_z"] = swdata->sensors[i].gyro_z;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/temperature_imu"] = swdata->sensors[i].temperature_imu;
                    root[variable_names_[var_num] + "/" + std::to_string(i+1) + "/pressure"] = swdata->sensors[i].pressure;
                }
                var_num += 1;
                root[variable_names_[var_num++]] = swdata->working_count;

                std::stringstream msg;
                msg << root;

                data_tunnel_interface_.tunnelData(readers::DataReader::formatVariableName(config::DataSourceNames::ROS, topic_name_),
                                                  msg.str());
            }
            catch (topic_tools::ShapeShifterException e)
            {
                std::cerr << "Shape shifter exception for message type ropod_ros_msgs::SmartWheelData" << std::endl;
                exit(0);
            }
        }
    }
}
