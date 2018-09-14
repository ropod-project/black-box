#include "datalogger/data_readers/zmq_reader.hpp"
#include <iostream>
#include <thread>

namespace readers
{
    /**
     * @param config_params ZMQ-specific configuration parameterss
     * @param data_logger a data logger instance for logging the received data
     */
    ZMQReader::ZMQReader(const config::ZmqTopicParams &config_params, std::shared_ptr<loggers::DataLogger> data_logger)
        : DataReader(data_logger), config_params_(config_params),
          zmq_context_(1), zmq_subscriber_(zmq_context_, ZMQ_SUB)
    {
    }

    ZMQReader::~ZMQReader()
    {
    }

    void ZMQReader::start()
    {
       zmq_subscriber_.connect(std::string(config_params_.url + ":" + std::to_string(config_params_.port)).c_str());
       zmq_subscriber_.setsockopt(ZMQ_SUBSCRIBE, config_params_.name.c_str(), 0);
       std::thread t(&ZMQReader::receiveLoop, this);
       t.detach();
    }

    /**
     * Does nothing at the moment
     */
    void ZMQReader::stop()
    {
    }

    /**
     * Callback executed when a ZMQ message is received
     */
    void ZMQReader::receiveLoop()
    {
        zmq::message_t zmq_message;

        // if topic name is not empty, it is an envelope message
        bool envelope_message = !config_params_.name.empty();

        while (true)
        {
            if (zmq_subscriber_.recv(&zmq_message, ZMQ_NOBLOCK))
            {
                // this means we only received the envelope
                // read again to get the actual message
                if (envelope_message)
                {
                    zmq_subscriber_.recv(&zmq_message);
                }
                parseMessage(zmq_message);
            }
        }
    }
}
