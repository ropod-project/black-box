#include "config/config_enums.hpp"

namespace config
{
    string ConfigKeys::DEFAULT_PARAMETERS = "default_parameters";
    string ConfigKeys::ROS = "ros";
    string ConfigKeys::ZMQ = "zmq";
    string ConfigKeys::ETHERCAT = "ethercat";
    string ConfigKeys::ZYRE = "zyre";

    string EthercatDataTypes::UINT8 = "uint8";
    string EthercatDataTypes::UINT16 = "uint16";
    string EthercatDataTypes::UINT32 = "uint32";
    string EthercatDataTypes::UINT64 = "uint64";
    string EthercatDataTypes::INT8 = "int8";
    string EthercatDataTypes::INT16 = "int16";
    string EthercatDataTypes::INT32 = "int32";
    string EthercatDataTypes::INT64 = "int64";
    string EthercatDataTypes::FLOAT = "float";

    string DataSourceNames::ROS = "ros";
    string DataSourceNames::ZMQ = "zmq";
    string DataSourceNames::ETHERCAT = "ethercat";
    string DataSourceNames::ZYRE = "zyre";
}
