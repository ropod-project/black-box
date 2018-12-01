#include <soem/ethercattype.h>
#include "datalogger/data_readers/ethercat_reader.hpp"
#include <iostream>

namespace readers
{
    /**
     * @param config_params ethercat-specific configuration parameterss
     * @param data_logger a data logger instance for logging the received data
     */
    EthercatReader::EthercatReader(const config::EthercatParams& config_params, std::shared_ptr<loggers::DataLogger> data_logger)
        : DataReader(data_logger), config_params_(config_params), sniffer_(config_params.interface)
    {
        // only accept ethercat messages
        sniffer_.set_filter("ether proto 0x88a4");
    }

    EthercatReader::~EthercatReader()
    {
    }

    /**
     * Starts the data reader
     */
    void EthercatReader::start()
    {
        std::thread t(&EthercatReader::startSniffer, this);
        t.detach();
    }

    /**
     * Stops the data reader
     */
    void EthercatReader::stop()
    {
    }

    /**
     * Starts an ethercat packet sniffer
     */
    void EthercatReader::startSniffer()
    {
        sniffer_.sniff_loop(std::bind(&EthercatReader::packetCallback, this, std::placeholders::_1));
    }

    /**
     * Callback executed when an ethercat packet is received
     *
     * @param pdu
     */
    bool EthercatReader::packetCallback(Tins::PDU &pdu)
    {
        Tins::EthernetII &eth  = pdu.rfind_pdu<Tins::EthernetII>();
        Tins::PDU::serialization_type buffer = eth.serialize();

        ec_comt ethercat_header;
        std::memcpy(&ethercat_header, &buffer[0] + eth.header_size(), sizeof(ec_comt));

        int datagram_size = ((int)(ethercat_header.dlength) & 0x0fff);

        parseDatagram(buffer, eth.header_size() + sizeof(ec_comt), datagram_size);
        return true;
    }

    /**
     * Parses a received ethercat packet
     *
     * @param buffer
     * @param start_offset
     * @param datagram_size
     */
    void EthercatReader::parseDatagram(const Tins::PDU::serialization_type &buffer, int start_offset, int datagram_size)
    {
        struct timeval tp;
        gettimeofday(&tp, NULL);
        float timestamp = (tp.tv_sec * 1000 + tp.tv_usec / 1000) / 1000.0;
        std::vector<double> output_values;
        std::vector<double> input_values;

        for (int i = 0; i < config_params_.number_of_slaves; i++)
        {
            int skip = start_offset + (i * config_params_.input_message_size);
            for (int j = 0; j < config_params_.input_message_elements.size(); j++)
            {
                if (config_params_.input_message_elements[j].type == config::EthercatDataTypes::UINT8)
                {
                    uint8_t value;
                    std::memcpy(&value, &buffer[0] + skip, sizeof(uint8_t));
                    input_values.push_back(value);
                    skip += sizeof(uint8_t);
                }
                else if (config_params_.input_message_elements[j].type == config::EthercatDataTypes::UINT16)
                {
                    uint16_t value;
                    std::memcpy(&value, &buffer[0] + skip, sizeof(uint16_t));
                    input_values.push_back(value);
                    skip += sizeof(uint16_t);
                }
                else if (config_params_.input_message_elements[j].type == config::EthercatDataTypes::UINT32)
                {
                    uint32_t value;
                    std::memcpy(&value, &buffer[0] + skip, sizeof(uint32_t));
                    input_values.push_back(value);
                    skip += sizeof(uint32_t);
                }
                else if (config_params_.input_message_elements[j].type == config::EthercatDataTypes::UINT64)
                {
                    uint64_t value;
                    std::memcpy(&value, &buffer[0] + skip, sizeof(uint64_t));
                    input_values.push_back(value);
                    skip += sizeof(uint64_t);
                }
                else if (config_params_.input_message_elements[j].type == config::EthercatDataTypes::INT8)
                {
                    int8_t value;
                    std::memcpy(&value, &buffer[0] + skip, sizeof(int8_t));
                    input_values.push_back(value);
                    skip += sizeof(int8_t);
                }
                else if (config_params_.input_message_elements[j].type == config::EthercatDataTypes::INT16)
                {
                    int16_t value;
                    std::memcpy(&value, &buffer[0] + skip, sizeof(int16_t));
                    input_values.push_back(value);
                    skip += sizeof(int16_t);
                }
                else if (config_params_.input_message_elements[j].type == config::EthercatDataTypes::INT32)
                {
                    int32_t value;
                    std::memcpy(&value, &buffer[0] + skip, sizeof(int32_t));
                    input_values.push_back(value);
                    skip += sizeof(int32_t);
                }
                else if (config_params_.input_message_elements[j].type == config::EthercatDataTypes::INT64)
                {
                    int64_t value;
                    std::memcpy(&value, &buffer[0] + skip, sizeof(int64_t));
                    input_values.push_back(value);
                    skip += sizeof(int64_t);
                }
                else if (config_params_.input_message_elements[j].type == config::EthercatDataTypes::FLOAT)
                {
                    float value;
                    std::memcpy(&value, &buffer[0] + skip, sizeof(float));
                    input_values.push_back(value);
                    skip += sizeof(float);
                }
            }
        }

        for (int i = 0; i < config_params_.number_of_slaves; i++)
        {
            int skip = start_offset
                       + (config_params_.number_of_slaves * config_params_.input_message_size)
                       + (i * config_params_.output_message_size);
            for (int j = 0; j < config_params_.output_message_elements.size(); j++)
            {
                if (config_params_.output_message_elements[j].type == config::EthercatDataTypes::UINT8)
                {
                    uint8_t value;
                    std::memcpy(&value, &buffer[0] + skip, sizeof(uint8_t));
                    output_values.push_back(value);
                    skip += sizeof(uint8_t);
                }
                else if (config_params_.output_message_elements[j].type == config::EthercatDataTypes::UINT16)
                {
                    uint16_t value;
                    std::memcpy(&value, &buffer[0] + skip, sizeof(uint16_t));
                    output_values.push_back(value);
                    skip += sizeof(uint16_t);
                }
                else if (config_params_.output_message_elements[j].type == config::EthercatDataTypes::UINT32)
                {
                    uint32_t value;
                    std::memcpy(&value, &buffer[0] + skip, sizeof(uint32_t));
                    output_values.push_back(value);
                    skip += sizeof(uint32_t);
                }
                else if (config_params_.output_message_elements[j].type == config::EthercatDataTypes::UINT64)
                {
                    uint64_t value;
                    std::memcpy(&value, &buffer[0] + skip, sizeof(uint64_t));
                    output_values.push_back(value);
                    skip += sizeof(uint64_t);
                }
                else if (config_params_.output_message_elements[j].type == config::EthercatDataTypes::INT8)
                {
                    int8_t value;
                    std::memcpy(&value, &buffer[0] + skip, sizeof(int8_t));
                    output_values.push_back(value);
                    skip += sizeof(int8_t);
                }
                else if (config_params_.output_message_elements[j].type == config::EthercatDataTypes::INT16)
                {
                    int16_t value;
                    std::memcpy(&value, &buffer[0] + skip, sizeof(int16_t));
                    output_values.push_back(value);
                    skip += sizeof(int16_t);
                }
                else if (config_params_.output_message_elements[j].type == config::EthercatDataTypes::INT32)
                {
                    int32_t value;
                    std::memcpy(&value, &buffer[0] + skip, sizeof(int32_t));
                    output_values.push_back(value);
                    skip += sizeof(int32_t);
                }
                else if (config_params_.output_message_elements[j].type == config::EthercatDataTypes::INT64)
                {
                    int64_t value;
                    std::memcpy(&value, &buffer[0] + skip, sizeof(int64_t));
                    output_values.push_back(value);
                    skip += sizeof(int64_t);
                }
                else if (config_params_.output_message_elements[j].type == config::EthercatDataTypes::FLOAT)
                {
                    float value;
                    std::memcpy(&value, &buffer[0] + skip, sizeof(float));
                    output_values.push_back(value);
                    skip += sizeof(float);
                }
            }
        }

        data_tunnel_interface_.tunnelData(DataReader::formatVariableName(config::DataSourceNames::ETHERCAT, config_params_.input_data_name),
                                          timestamp,
                                          input_values);

        data_tunnel_interface_.tunnelData(DataReader::formatVariableName(config::DataSourceNames::ETHERCAT, config_params_.output_data_name),
                                          timestamp,
                                          output_values);
    }
}
