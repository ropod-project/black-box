#ifndef DATA_TUNNEL_INTERFACE_H
#define DATA_TUNNEL_INTERFACE_H

#include <memory>
#include "datalogger/data_loggers/data_logger.hpp"
#include <json/json.h>

namespace readers
{
    /**
     * An interface for tunneling data to multiple endpoints.
     *
     * @author Alex Mitrevski, Santosh Thoduka
     * @contact aleksandar.mitrevski@h-brs.de, santosh.thoduka@h-brs.de
     */
    class DataTunnelInterface
    {
    public:
        DataTunnelInterface(std::shared_ptr<loggers::DataLogger> data_logger);

        void tunnelData(const std::string &variable_name, double timestamp, const std::vector<int> &values);
        void tunnelData(const std::string &variable_name, double timestamp, const std::vector<float> &values);
        void tunnelData(const std::string &variable_name, double timestamp, const std::vector<double> &values);
        void tunnelData(const std::string &variable_name, double timestamp, const std::vector<bool> &values);
        void tunnelData(const std::string &variable_name, double timestamp, const std::vector<std::string> &values);
        void tunnelData(const std::string &variable_name, double timestamp, const std::vector<std::vector<float>> &values);
        void tunnelData(const std::string &variable_name, double timestamp, const std::vector<std::vector<std::vector<float>>> &values);
        void tunnelData(const std::string &variable_name, double timestamp, const std::vector<float> &values, const std::vector<std::string> &str_values);
        void tunnelData(const std::string &variable_name, double timestamp, const std::string &json_string);
        void tunnelData(const std::string &variable_name, const std::string &str);
    private:
        std::shared_ptr<loggers::DataLogger> data_logger_;
    };
}

#endif
