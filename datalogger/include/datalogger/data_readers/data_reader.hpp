#ifndef DATA_READER_H
#define DATA_READER_H

#include <vector>
#include <string>
#include <map>
#include <memory>

#include "config/config_enums.hpp"
#include "datalogger/data_readers/data_tunnel_interface.h"
#include "datalogger/data_loggers/data_logger.hpp"

namespace readers
{
    /**
     * An interface for receiving data from a given source
     *
     * @author Alex Mitrevski, Santosh Thoduka
     * @contact aleksandar.mitrevski@h-brs.de, santosh.thoduka@h-brs.de
     */
    class DataReader
    {
    public:
        /**
         * @param data_logger a data logger instance for logging the received data
         */
        DataReader(std::shared_ptr<loggers::DataLogger> data_logger);
        virtual ~DataReader() {}

        /**
         * Starts the data reader
         */
        virtual void start() = 0;

        /**
         * Stops the data reader
         */
        virtual void stop() = 0;

        // bool addVariable(int variable, const std::string &variable_name);

        /**
         * Returns a formatted string that contains the given source name
         * and variable name in the form 'sourceName_variableName'
         *
         * @param source_name name of a data source
         * @param variable_name name of a variable
         */
        static std::string formatVariableName(std::string source_name, std::string variable_name);

    protected:
        DataTunnelInterface data_tunnel_interface_;
    };
}

#endif /* DATA_READER_H */
