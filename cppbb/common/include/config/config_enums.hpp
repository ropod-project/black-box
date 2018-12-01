#ifndef CONFIG_ENUMS_H
#define CONFIG_ENUMS_H

#include <string>
using std::string;

namespace config
{
    /**
     * Top-level keys allowed in a black box config file
     *
     * @author Alex Mitrevski, Santosh Thoduka
     * @contact aleksandar.mitrevski@h-brs.de, santosh.thoduka@h-brs.de
     */
    struct ConfigKeys
    {
        static string DEFAULT_PARAMETERS;
        static string ROS;
        static string ZMQ;
        static string ZYRE;
        static string ETHERCAT;
    };

    /**
     * Allowed data types for ethercat data
     *
     * @author Alex Mitrevski, Santosh Thoduka
     * @contact aleksandar.mitrevski@h-brs.de, santosh.thoduka@h-brs.de
     */
    struct EthercatDataTypes
    {
        static string UINT8;
        static string UINT16;
        static string UINT32;
        static string UINT64;
        static string INT8;
        static string INT16;
        static string INT32;
        static string INT64;
        static string FLOAT;
    };

    /**
     * Names of data sources a black box is reading from
     *
     * @author Alex Mitrevski, Santosh Thoduka
     * @contact aleksandar.mitrevski@h-brs.de, santosh.thoduka@h-brs.de
     */
    struct DataSourceNames
    {
        static string ROS;
        static string ZMQ;
        static string ETHERCAT;
        static string ZYRE;
    };
}

#endif /* CONFIG_ENUMS_H */
