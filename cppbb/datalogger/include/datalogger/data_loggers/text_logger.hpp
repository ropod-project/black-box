#ifndef TEXT_LOGGER_HPP
#define TEXT_LOGGER_HPP

#include <fstream>
#include <map>
#include <memory>

#include "datalogger/data_loggers/data_logger.hpp"

namespace loggers
{
    /**
     * An interface for logging data into text files
     *
     * @author Alex Mitrevski, Santosh Thoduka
     * @contact aleksandar.mitrevski@h-brs.de, santosh.thoduka@h-brs.de
     */
    class TextLogger : public DataLogger
    {
    public:
        /**
         * @param group_names a list of variable group names
         * @param variable_names a two-dimensional list of variable names corresponding to the group names
         */
        TextLogger(const std::vector<std::string> &group_names, const std::map<std::string, std::vector<std::string>> &variable_names);
        virtual ~TextLogger ();

        // virtual bool addVariable(const std::string &variable_name);

        /**
         * Returns a list of the names of all variables
         * corresponding to the given data source
         *
         * @param data_source_name name of a data source
         */
        virtual std::vector<std::string> getVariables(std::string data_source_name);

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
                                                                        const std::string end_time);

        /**
         * Logs a list of integers
         *
         * @param variable name of the variable to log
         * @param timestamp data timestamp
         * @param values a list of integers that should be logged
         */
        virtual void writeLog(std::string variable, double timestamp, const std::vector<int> &values);

        /**
         * Logs a list of floating-point values
         *
         * @param variable name of the variable to log
         * @param timestamp data timestamp
         * @param values a list of floats that should be logged
         */
        virtual void writeLog(std::string variable, double timestamp, const std::vector<float> &values);

        /**
         * Logs a list of doubles
         *
         * @param variable name of the variable to log
         * @param timestamp data timestamp
         * @param values a list of doubles that should be logged
         */
        virtual void writeLog(std::string variable, double timestamp, const std::vector<double> &values);

        /**
         * Logs a list of Boolean values
         *
         * @param variable name of the variable to log
         * @param timestamp data timestamp
         * @param values a list of booleans that should be logged
         */
        virtual void writeLog(std::string variable, double timestamp, const std::vector<bool> &values);

        /**
         * Logs a list of strings
         *
         * @param variable name of the variable to log
         * @param timestamp data timestamp
         * @param values a list of strings that should be logged
         */
        virtual void writeLog(std::string variable, double timestamp, const std::vector<std::string> &values);

        /**
         * Logs a two-dimensional list of floating-point values
         *
         * @param variable name of the variable to log
         * @param timestamp data timestamp
         * @param values a list of integers that should be logged
         */
        virtual void writeLog(std::string variable, double timestamp, const std::vector<std::vector<float>> &values);

        /**
         * Logs a json string
         *
         * @param variable name of the variable to log
         * @param timestamp data timestamp
         * @param json_string a string that should be logged
         */
        virtual void writeLog(std::string variable, double timestamp, const std::string &json_string);

    private:
        /**
         * Returns the name of a txt file with the given variable name
         * (i.e. the name is just '<variable_name>.txt')
         *
         * @param variable name name of a variable
         */
        std::string getFileName(const std::string &variable_name);
        std::map<std::string, std::shared_ptr<std::ofstream> > file_writers_;
    };
}

#endif
