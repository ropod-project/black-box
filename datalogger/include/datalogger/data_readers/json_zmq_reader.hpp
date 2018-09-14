#ifndef JSON_ZMQ_READER_H
#define JSON_ZMQ_READER_H

#include <zmq.hpp>
#include <json/json.h>

#include "datalogger/data_readers/zmq_reader.hpp"

namespace readers
{
    /**
     * An interface for receiving ZMQ data
     *
     * @author Alex Mitrevski, Santosh Thoduka
     * @contact aleksandar.mitrevski@h-brs.de, santosh.thoduka@h-brs.de
     */
    class JsonZMQReader : public ZMQReader
    {
    public:
        /**
         * @param config_params ZMQ-specific configuration parameterss
         * @param data_logger a data logger instance for logging the received data
         */
        JsonZMQReader(const config::ZmqTopicParams &config_params, std::shared_ptr<loggers::DataLogger> data_logger);
        virtual ~JsonZMQReader();

    protected:
        /**
         * Parses a received ZMQ message
         *
         * @param zmq_message received message
         */
        void parseMessage(zmq::message_t &zmq_message);
        Json::CharReaderBuilder rbuilder_;
        std::unique_ptr<Json::CharReader> reader_;
    };
}

#endif /* JSON_ZMQ_READER_H */
