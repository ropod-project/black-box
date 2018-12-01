#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <vector>
#include <map>
#include <string>

namespace loggers
{
    /**
     * An interface for data logging functionalities
     *
     * @author Alex Mitrevski, Santosh Thoduka
     * @contact aleksandar.mitrevski@h-brs.de, santosh.thoduka@h-brs.de
     */
    class DataLogger
    {
    public:
        DataLogger(){}
        virtual ~DataLogger() {}

        // virtual bool addVariable(int variable, const std::string &variable_name) = 0;

        /**
         * Returns a list of the names of all variables
         * corresponding to the given data source
         *
         * @param data_source_name name of a data source
         */
        virtual std::vector<std::string> getVariables(std::string data_source_name) = 0;

        /**
         * Returns a data dictionary in which each key represents a variable
         * and the value represents data corresponding to that variable
         * in a given time interval
         *
         * @param variables a list of variable names
         * @param start_time a string representing the start data timestamp
         * @param end_time a string representing the end data timestamp
         */
        virtual std::map<std::string, std::vector<std::string>> getData(const std::map<std::string, std::vector<std::string>> &variables,
                                                                        const std::string start_time,
                                                                        const std::string end_time) = 0;

        /**
         * Logs a list of integers
         *
         * @param variable name of the variable to log
         * @param timestamp data timestamp
         * @param values a list of integers that should be logged
         */
        virtual void writeLog(std::string variable, double timestamp, const std::vector<int> &values) = 0;

        /**
         * Logs a list of floating-point values
         *
         * @param variable name of the variable to log
         * @param timestamp data timestamp
         * @param values a list of floats that should be logged
         */
        virtual void writeLog(std::string variable, double timestamp, const std::vector<float> &values) = 0;

        /**
         * Logs a list of doubles
         *
         * @param variable name of the variable to log
         * @param timestamp data timestamp
         * @param values a list of doubles that should be logged
         */
        virtual void writeLog(std::string variable, double timestamp, const std::vector<double> &values) = 0;

        /**
         * Logs a list of Boolean values
         *
         * @param variable name of the variable to log
         * @param timestamp data timestamp
         * @param values a list of booleans that should be logged
         */
        virtual void writeLog(std::string variable, double timestamp, const std::vector<bool> &values) = 0;

        /**
         * Logs a list of strings
         *
         * @param variable name of the variable to log
         * @param timestamp data timestamp
         * @param values a list of strings that should be logged
         */
        virtual void writeLog(std::string variable, double timestamp, const std::vector<std::string> &values) = 0;

        /**
         * Logs a two-dimensional list of floating-point values
         *
         * @param variable name of the variable to log
         * @param timestamp data timestamp
         * @param values a list of integers that should be logged
         */
        virtual void writeLog(std::string variable, double timestamp, const std::vector<std::vector<float>> &values) = 0;

        /**
         * Logs a three-dimensional list of floating-point values
         *
         * @param variable name of the variable to log
         * @param timestamp data timestamp
         * @param values a list of three dimensional floats that should be logged
         */
        virtual void writeLog(std::string variable, double timestamp, const std::vector<std::vector<std::vector<float>>> &values) = 0;

        /**
         * Logs a list of floats and a list of strings
         *
         * @param variable name of the variable to log
         * @param timestamp data timestamp
         * @param values a list of floats  that should be logged
         * @param str_values a list of strings that should be logged
         */
        virtual void writeLog(std::string variable, double timestamp,
                const std::vector<float> &values, const std::vector<std::string> &str_values) = 0;


        /**
         * Logs a json string
         *
         * @param variable name of the variable to log
         * @param timestamp data timestamp
         * @param json_string a string that should be logged
         */
        virtual void writeLog(std::string variable, double timestamp, const std::string &json_string) = 0;

        /**
         * Logs a json string
         *
         * @param variable name of the variable to log
         * @param json_string a string in json format that should be logged; the timestamp is part of the string
         */
        virtual void writeLog(std::string variable, const std::string &str) = 0;

    protected:
        std::vector<std::string> group_names_;
        std::map<std::string, std::vector<std::string>> variable_names_;
    };
}

#endif /* DATA_LOGGER_H */
