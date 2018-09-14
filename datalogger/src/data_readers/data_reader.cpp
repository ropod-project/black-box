#include "datalogger/data_readers/data_reader.hpp"
#include <algorithm>

namespace readers
{
    /**
     * @param data_logger a data logger instance for logging the received data
     */
    DataReader::DataReader(std::shared_ptr<loggers::DataLogger> data_logger)
        : data_tunnel_interface_(data_logger)
    {
    }

    /**
     * Returns a formatted string that contains the given source name
     * and variable name in the form 'sourceName_variableName'
     *
     * @param source_name name of a data source
     * @param variable_name name of a variable
     */
    std::string DataReader::formatVariableName(std::string source_name, std::string variable_name)
    {
        if (variable_name[0] == '/')
        {
            variable_name = variable_name.substr(1, variable_name.size()-1);
        }
        return source_name + "_" + variable_name;
    }
}
