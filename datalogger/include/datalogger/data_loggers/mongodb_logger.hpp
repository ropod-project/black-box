#ifndef MONGODB_LOGGER_HPP
#define MONGODB_LOGGER_HPP

#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/json.hpp>

#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>

#include <boost/algorithm/string/predicate.hpp>

#include "datalogger/data_loggers/data_logger.hpp"
#include <json/json.h>

namespace loggers
{
    /**
     * An interface for logging data into a MongoDB database
     *
     * @author Alex Mitrevski, Santosh Thoduka
     * @contact aleksandar.mitrevski@h-brs.de, santosh.thoduka@h-brs.de
     */
    class MongodbLogger : public DataLogger
    {
    public:
        /**
         * @param db_name_prefix prefix for the name of a database
         * @param group_names a list of variable group names
         * @param variable_names a two-dimensional list of variable names corresponding to the group names
         */
        MongodbLogger(std::string db_name_prefix, bool split_database, unsigned int max_database_size,
                      const std::vector<std::string> &group_names, const std::map<std::string, std::vector<std::string>> &variable_names);

        /**
         * Returns the size of the database (in MB) that is currently used by the logger
         *
         * @return database size in MB
         */
        unsigned int getDatabaseSize();

        /**
         * Returns the name of the database with the latest timestamp in the name
         *
         * @return name of database with the latest timestamp
         */
        std::string getLatestDatabaseName();

        /**
         * Generates a new database name based on the prefix and current timestamp
         *
         * @return new name of database
         */
        std::string createNewDatabaseName();

        /**
         * If current database has reached maximum size,
         * creates a new database and sets it to be the current one
         *
         */
        void setCurrentDb();

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
         * Logs a three-dimensional list of floating-point values
         *
         * @param variable name of the variable to log
         * @param timestamp data timestamp
         * @param values a list of three dimensional floats that should be logged
         */
        virtual void writeLog(std::string variable, double timestamp, const std::vector<std::vector<std::vector<float>>> &values);

        /**
         * Logs a list of floats and a list of strings
         *
         * @param variable name of the variable to log
         * @param timestamp data timestamp
         * @param values a list of floats  that should be logged
         * @param str_values a list of strings that should be logged
         */
        virtual void writeLog(std::string variable, double timestamp,
                const std::vector<float> &values, const std::vector<std::string> &str_values);

        /**
         * Logs a json string
         *
         * @param variable name of the variable to log
         * @param timestamp data timestamp
         * @param json_string a string that should be logged
         */
        virtual void writeLog(std::string variable, double timestamp, const std::string &json_string);

        /**
         * Logs a json object
         *
         * @param variable name of the variable to log
         * @param json_string a string in json format that should be logged; the timestamp is part of the string
         */
        virtual void writeLog(std::string variable, const std::string &str);

    private:
        /**
         * The prefix to the database name
         * A typical database name will include the prefix
         * and a timestamp of when the database was first
         * created
         * Example: logs_20180328150208
         */
        std::string db_name_prefix_;

        /**
         * Full name of the current database being written to
         */
        std::string db_name_;

        /**
         * Maximum allowable database size on MB
         */
        unsigned int max_database_size_;
        /**
         * Whether or not to create a new database when max_database_size is reached
         */
        bool split_database_;
        mongocxx::instance instance_;
    };
}

#endif
