#include "ethercat_query_interface.h"

namespace data_retrieval
{
    /**
     * Returns a list of the names of all ethercat variables
     *
     * @param data_logger a DataLogger shared pointer
     */
    std::vector<std::string> EthercatQueryInterface::getVariables(const std::shared_ptr<loggers::DataLogger> data_logger)
    {
        std::vector<std::string> variables = data_logger->getVariables(config::DataSourceNames::ETHERCAT);
        return variables;
    }

    /**
     * Returns a data dictionary in which each key represents an ethercat variable
     * and the value represents data corresponding to that variable
     * in a given time interval
     *
     * @param data_logger a DataLogger shared pointer
     * @param variables a list of variable names
     * @param start_time a string representing the start data timestamp
     * @param end_time a string representing the end data timestamp
     */
    std::map<std::string, std::vector<std::string>> EthercatQueryInterface::getData(const std::shared_ptr<loggers::DataLogger> data_logger,
                                                                                    const std::map<std::string, std::vector<std::string>> &variables,
                                                                                    const std::string start_time,
                                                                                    const std::string end_time)
    {
        std::map<std::string, std::vector<std::string>> data = data_logger->getData(variables, start_time, end_time);
        return data;
    }
}
