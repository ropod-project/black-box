#include "datalogger/data_readers/data_tunnel_interface.h"

namespace readers
{
    DataTunnelInterface::DataTunnelInterface(std::shared_ptr<loggers::DataLogger> data_logger)
        : data_logger_(data_logger)
        {}

    void DataTunnelInterface::tunnelData(const std::string &variable_name, double timestamp, const std::vector<int> &values)
    {
        data_logger_->writeLog(variable_name, timestamp, values);
    }

    void DataTunnelInterface::tunnelData(const std::string &variable_name, double timestamp, const std::vector<float> &values)
    {
        data_logger_->writeLog(variable_name, timestamp, values);
    }

    void DataTunnelInterface::tunnelData(const std::string &variable_name, double timestamp, const std::vector<double> &values)
    {
        data_logger_->writeLog(variable_name, timestamp, values);
    }

    void DataTunnelInterface::tunnelData(const std::string &variable_name, double timestamp, const std::vector<bool> &values)
    {
        data_logger_->writeLog(variable_name, timestamp, values);
    }

    void DataTunnelInterface::tunnelData(const std::string &variable_name, double timestamp, const std::vector<std::string> &values)
    {
        data_logger_->writeLog(variable_name, timestamp, values);
    }

    void DataTunnelInterface::tunnelData(const std::string &variable_name, double timestamp, const std::vector<std::vector<float>> &values)
    {
        data_logger_->writeLog(variable_name, timestamp, values);
    }

    void DataTunnelInterface::tunnelData(const std::string &variable_name, double timestamp, const std::vector<std::vector<std::vector<float>>> &values)
    {
        data_logger_->writeLog(variable_name, timestamp, values);
    }

    void DataTunnelInterface::tunnelData(const std::string &variable_name, double timestamp,
            const std::vector<float> &values, const std::vector<std::string> &str_values)
    {
        data_logger_->writeLog(variable_name, timestamp, values, str_values);
    }

    void DataTunnelInterface::tunnelData(const std::string &variable_name, double timestamp, const std::string &json_string)
    {
        data_logger_->writeLog(variable_name, timestamp, json_string);
    }

    void DataTunnelInterface::tunnelData(const std::string &variable_name, const std::string &str)
    {
        data_logger_->writeLog(variable_name, str);
    }
}
