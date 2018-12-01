#ifndef ROSTOPIC_READER_H
#define ROSTOPIC_READER_H

#include <ros/ros.h>

#include "config/config_params.hpp"
#include "datalogger/data_readers/data_reader.hpp"
#include "datalogger/data_loggers/data_logger.hpp"
#include "datalogger/rostopic_listeners/rostopic_listener_factory.hpp"
#include "datalogger/rostopic_listeners/rostopic_listener_base.hpp"

namespace readers
{
    /**
     * An interface for receiving ROS topic data
     *
     * @author Alex Mitrevski, Santosh Thoduka
     * @contact aleksandar.mitrevski@h-brs.de, santosh.thoduka@h-brs.de
     */
    class ROSTopicReader : public DataReader
    {
    public:
        /**
         * @param argc number of command line arguments
         * @param argv command line arguments
         * @param node_handle_name name of a ROS node that subscribes to different topics
         * @param config_params ROS-specific configuration parameterss
         * @param max_frequency maximum frequency at which to log messages (decides loop rate)
         * @param data_logger a data logger instance for logging the received data
         */
        ROSTopicReader(int argc, char **argv,
                       std::string node_handle_name,
                       const config::RosParams& config_params,
                       double max_frequency,
                       std::shared_ptr<loggers::DataLogger> data_logger);

       /**
        * Shuts down all subscribers
        */
        virtual ~ROSTopicReader();

        /**
         * Starts the data reader
         */
        void start();

        /**
         * Stops the data reader
         */
        void stop();

        /**
         * Runs the main node loop
         */
        void loop();
    private:
        /**
         * Starts a new ROS node
         */
        void startNode();

        /**
         * Creates a set of subscribers for different ROS topics
         *
         * @param data_logger a data logger instance for logging the received data
         */
        void createListeners(std::shared_ptr<loggers::DataLogger> data_logger);

        int argc_;
        char **argv_;
        std::string node_handle_name_;
        config::RosParams config_params_;
        double max_frequency_;
        std::vector<std::shared_ptr<ros_listeners::ROSTopicListenerBase>> readers_;
        std::shared_ptr<ros::NodeHandle> nh_;
    };
}

#endif /* ROSTOPIC_READER_H */
