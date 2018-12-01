#ifndef ZMQ_READER_H
#define ZMQ_READER_H

#include <string>
#include <memory>
#include <zmq.hpp>

#include "config/config_params.hpp"
#include "config/config_enums.hpp"
#include "datalogger/data_readers/data_reader.hpp"
#include "datalogger/data_loggers/data_logger.hpp"

namespace readers
{
    /**
     * An interface for receiving ZMQ data
     *
     * @author Alex Mitrevski, Santosh Thoduka
     * @contact aleksandar.mitrevski@h-brs.de, santosh.thoduka@h-brs.de
     */
    class ZMQReader : public DataReader
    {
    public:
        /**
         * @param config_params ZMQ-specific configuration parameterss
         * @param data_logger a data logger instance for logging the received data
         */
        ZMQReader(const config::ZmqTopicParams &config_params, std::shared_ptr<loggers::DataLogger> data_logger);
        virtual ~ZMQReader();

        /**
         * Starts a ZMQ subscriber
         */
        void start();

        /**
         * Does nothing at the moment
         */
        void stop();

        /**
         * Callback executed when a ZMQ message is received
         */
        void receiveLoop();
    protected:
        /**
         * Parses a received ZMQ message
         *
         * @param zmq_message received message
         */
        virtual void parseMessage(zmq::message_t &zmq_message) = 0;

        config::ZmqTopicParams config_params_;

        zmq::context_t zmq_context_;
        zmq::socket_t zmq_subscriber_;

        std::string ip_address_;
        int port_;
    };
}

#endif /* ZMQ_READER_H */
