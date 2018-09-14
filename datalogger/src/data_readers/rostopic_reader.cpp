#include "datalogger/data_readers/rostopic_reader.hpp"

namespace readers
{
    /**
     * @param argc number of command line arguments
     * @param argv command line arguments
     * @param node_handle_name name of a ROS node that subscribes to different topics
     * @param config_params ROS-specific configuration parameterss
     * @param data_logger a data logger instance for logging the received data
     */
    ROSTopicReader::ROSTopicReader(int argc, char **argv, std::string node_handle_name, const config::RosParams& config_params, double max_frequency, std::shared_ptr<loggers::DataLogger> data_logger)
        : DataReader(data_logger), config_params_(config_params),
          max_frequency_(max_frequency), node_handle_name_(node_handle_name)
    {
        argc_ = argc;
        argv_ = argv;
        this->startNode();
        this->createListeners(data_logger);
    }

    /**
     * Shuts down all subscribers
     */
    ROSTopicReader::~ROSTopicReader()
    {
        for (int i = 0; i < readers_.size(); i++)
        {
            readers_[i]->shutdownSubscriber();
        }
    }

    /**
     * Starts a new ROS node
     */
    void ROSTopicReader::startNode()
    {
        ros::init(argc_, argv_, node_handle_name_);
        nh_.reset(new ros::NodeHandle());
    }

    /**
     * Starts the data reader
     */
    void ROSTopicReader::start()
    {
        for (int i = 0; i < config_params_.topics.size(); i++)
        {
            readers_[i]->createSubscriber(nh_);
        }
    }

    /**
     * Stops the data reader
     */
    void ROSTopicReader::stop()
    {
        for (int i = 0; i < readers_.size(); i++)
        {
            readers_[i]->shutdownSubscriber();
            readers_[i]->resetSubscriber();
        }
    }

    /**
     * Runs the main node loop
     */
    void ROSTopicReader::loop()
    {
        ros::Rate loop_rate(max_frequency_);
        while (ros::ok())
        {
            while (ros::ok() && ros::master::check())
            {
                ros::spinOnce();
                loop_rate.sleep();
            }
            std::cerr << "ROS master died. Attempting to reconnect" << std::endl;
            stop();
            startNode();
            ros::spinOnce();
            start();
        }
    }

    /**
     * Creates a set of subscribers for different ROS topics
     *
     * @param data_logger a data logger instance for logging the received data
     */
    void ROSTopicReader::createListeners(std::shared_ptr<loggers::DataLogger> data_logger)
    {
        for (unsigned int i=0; i<config_params_.topics.size(); i++)
        {
            readers_.push_back(ros_listeners::ROSTopicListenerFactory::createListener(
                        config_params_.topics[i].name, config_params_.topics[i].type,
                        config_params_.topics[i].variable_names,
                        config_params_.topics[i].max_frequency, data_logger));
        }
    }
}
