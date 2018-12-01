#ifndef CONFIG_FILE_READER_H
#define CONFIG_FILE_READER_H

#include <iostream>
#include <vector>
#include <string>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/node.h>
#include "config/config_enums.hpp"
#include "config/config_params.hpp"

namespace config
{
    /**
     * An interface for reading black box configuration files
     *
     * @author Alex Mitrevski, Santosh Thoduka
     * @contact aleksandar.mitrevski@h-brs.de, santosh.thoduka@h-brs.de
     */
    class ConfigFileReader
    {
    public:
        /**
         * Loads the configuration parameters of a black box from the given YAML file
         *
         * @param config_file_name absolute path of a config file
         */
        static ConfigParams load(const std::string config_file_name);
    };

    class ConfigException : public std::runtime_error
    {
    public:
        ConfigException(std::string message)
            : std::runtime_error(message.c_str()), message_(message) {}

        virtual const char* what() const throw()
        {
            return message_.c_str();
        }

    private:
        std::string message_;
    };
}

#endif /* CONFIG_FILE_READER_H */
