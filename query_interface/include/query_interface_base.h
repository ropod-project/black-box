#ifndef QUERY_INTERFACE_BASE_H
#define QUERY_INTERFACE_BASE_H

#include <memory>
#include <string>
#include <map>
#include <vector>
#include <json/json.h>

#include "datalogger/data_loggers/data_logger.hpp"
#include "config/config_params.hpp"
#include "config/config_enums.hpp"

namespace data_retrieval
{
    /**
     * An interface for answering data queries
     *
     * @author Alex Mitrevski, Santosh Thoduka
     * @contact aleksandar.mitrevski@h-brs.de, santosh.thoduka@h-brs.de
     */
    class QueryInterfaceBase
    {
    public:
        /**
         * Returns a list of the names of all variables
         *
         * @param data_logger a DataLogger shared pointer
         */
        virtual std::vector<std::string> getVariables(const std::shared_ptr<loggers::DataLogger> data_logger) = 0;

        /**
         * Returns a data dictionary in which each key represents a variable
         * and the value represents data corresponding to that variable
         * in a given time interval
         *
         * @param data_logger a DataLogger shared pointer
         * @param variables a list of variable names
         * @param start_time a string representing the start data timestamp
         * @param end_time a string representing the end data timestamp
         */
        virtual std::map<std::string, std::vector<std::string>> getData(const std::shared_ptr<loggers::DataLogger> data_logger,
                                                                        const std::map<std::string, std::vector<std::string>> &variables,
                                                                        const std::string start_time,
                                                                        const std::string end_time) = 0;
    };
}

#endif
