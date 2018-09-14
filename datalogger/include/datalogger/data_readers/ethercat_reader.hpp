#ifndef ETHERCAT_READER_H
#define ETHERCAT_READER_H

#include <thread>
#include <tins/tins.h>

#include "config/config_params.hpp"
#include "config/config_enums.hpp"
#include "datalogger/data_readers/data_reader.hpp"
#include "datalogger/data_loggers/data_logger.hpp"

namespace readers
{
    /**
     * An interface for receiving ethercat data
     *
     * @author Alex Mitrevski, Santosh Thoduka
     * @contact aleksandar.mitrevski@h-brs.de, santosh.thoduka@h-brs.de
     */
    class EthercatReader : public DataReader
    {
    public:
        /**
         * @param config_params ethercat-specific configuration parameterss
         * @param data_logger a data logger instance for logging the received data
         */
        EthercatReader(const config::EthercatParams& config_params, std::shared_ptr<loggers::DataLogger> data_logger);

        virtual ~EthercatReader();

        /**
         * Starts the data reader
         */
        void start();

        /**
         * Stops the data reader
         */
        void stop();

        bool addVariable(int variable, const std::string &variable_name);

    protected:
        /**
         * Starts an ethercat packet sniffer
         */
        void startSniffer();

        /**
         * Callback executed when an ethercat packet is received
         *
         * @param pdu
         */
        bool packetCallback(Tins::PDU &pdu);

        /**
         * Parses a received ethercat packet
         *
         * @param buffer
         * @param start_offset
         * @param datagram_size
         */
        virtual void parseDatagram(const Tins::PDU::serialization_type &buffer, int start_offset, int datagram_size);
    private:
        void readData(const Tins::PDU::serialization_type &buffer, int &start_offset);

        config::EthercatParams config_params_;
        Tins::Sniffer sniffer_;
    };
}

#endif /* ETHERCAT_READER_H */
