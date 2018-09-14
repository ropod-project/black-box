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
                Json::Value commands;
                for (int i = 0; i < swdata->commands.size(); i++)
                {
                    Json::Value com;
                    com["command1"] = swdata->commands[i].command1;
                    com["command2"] = swdata->commands[i].command2;
                    com["setpoint1"] = swdata->commands[i].setpoint1;
                    com["setpoint2"] = swdata->commands[i].setpoint2;
                    com["limit1_p"] = swdata->commands[i].limit1_p;
                    com["limit1_n"] = swdata->commands[i].limit1_n;
                    com["limit2_p"] = swdata->commands[i].limit2_p;
                    com["limit2_n"] = swdata->commands[i].limit2_n;
                    com["timestamp"] = swdata->commands[i].timestamp;

                    commands.append(com);
                }
                root[variable_names_[var_num++]] = commands;

                Json::Value sensors;
                for (int i = 0; i < swdata->sensors.size(); i++)
                {
                    Json::Value sens;
                    sens["status1"] = swdata->sensors[i].status1;
                    sens["status2"] = swdata->sensors[i].status2;
                    sens["sensor_ts"] = swdata->sensors[i].sensor_ts;
                    sens["setpoint_ts"] = swdata->sensors[i].setpoint_ts;
                    sens["encoder_1"] = swdata->sensors[i].encoder_1;
                    sens["velocity_1"] = swdata->sensors[i].velocity_1;
                    sens["current_1_d"] = swdata->sensors[i].current_1_d;
                    sens["current_1_q"] = swdata->sensors[i].current_1_q;
                    sens["current_1_u"] = swdata->sensors[i].current_1_u;
                    sens["current_1_v"] = swdata->sensors[i].current_1_v;
                    sens["current_1_w"] = swdata->sensors[i].current_1_w;
                    sens["voltage_1"] = swdata->sensors[i].voltage_1;
                    sens["voltage_1_u"] = swdata->sensors[i].voltage_1_u;
                    sens["voltage_1_v"] = swdata->sensors[i].voltage_1_v;
                    sens["voltage_1_w"] = swdata->sensors[i].voltage_1_w;
                    sens["temperature_1"] = swdata->sensors[i].temperature_1;
                    sens["encoder_2"] = swdata->sensors[i].encoder_2;
                    sens["velocity_2"] = swdata->sensors[i].velocity_2;
                    sens["current_2_d"] = swdata->sensors[i].current_2_d;
                    sens["current_2_q"] = swdata->sensors[i].current_2_q;
                    sens["current_2_u"] = swdata->sensors[i].current_2_u;
                    sens["current_2_v"] = swdata->sensors[i].current_2_v;
                    sens["current_2_w"] = swdata->sensors[i].current_2_w;
                    sens["voltage_2"] = swdata->sensors[i].voltage_2;
                    sens["voltage_2_u"] = swdata->sensors[i].voltage_2_u;
                    sens["voltage_2_v"] = swdata->sensors[i].voltage_2_v;
                    sens["voltage_2_w"] = swdata->sensors[i].voltage_2_w;
                    sens["temperature_2"] = swdata->sensors[i].temperature_2;
                    sens["encoder_pivot"] = swdata->sensors[i].encoder_pivot;
                    sens["velocity_pivot"] = swdata->sensors[i].velocity_pivot;
                    sens["voltage_bus"] = swdata->sensors[i].voltage_bus;
                    sens["imu_ts"] = swdata->sensors[i].imu_ts;
                    sens["accel_x"] = swdata->sensors[i].accel_x;
                    sens["accel_y"] = swdata->sensors[i].accel_y;
                    sens["accel_z"] = swdata->sensors[i].accel_z;
                    sens["gyro_x"] = swdata->sensors[i].gyro_x;
                    sens["gyro_y"] = swdata->sensors[i].gyro_y;
                    sens["gyro_z"] = swdata->sensors[i].gyro_z;
                    sens["temperature_imu"] = swdata->sensors[i].temperature_imu;
                    sens["pressure"] = swdata->sensors[i].pressure;
                    sensors.append(sens);
                }
                root[variable_names_[var_num++]] = sensors;

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
