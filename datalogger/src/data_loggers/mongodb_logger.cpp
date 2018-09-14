#include "datalogger/data_loggers/mongodb_logger.hpp"
#include <bsoncxx/json.hpp>
#include <bsoncxx/types.hpp>
#include <iostream>
#include <ctime>
#include <chrono>
#include <iomanip>

namespace loggers
{
    /**
     * @param db_name name of a database
     * @param group_names a list of variable group names
     * @param variable_names a two-dimensional list of variable names corresponding to the group names
     */
    MongodbLogger::MongodbLogger(std::string db_name_prefix, bool split_database, unsigned int max_database_size,
                                 const std::vector<std::string> &group_names, const std::map<std::string, std::vector<std::string>> &variable_names)
        : db_name_prefix_(db_name_prefix), split_database_(split_database), max_database_size_(max_database_size), instance_{}
    {
        group_names_ = group_names;
        variable_names_ = variable_names;
        if (split_database_)
        {
            db_name_  = getLatestDatabaseName();
        }
        else
        {
            db_name_ = db_name_prefix_;
        }
        std::cout << "Writing to DB: " << db_name_ << std::endl;
    }

    /**
     * Returns the size of the database (in MB) that is currently used by the logger
     */
    unsigned int MongodbLogger::getDatabaseSize()
    {
        mongocxx::client db_client{mongocxx::uri{}};
        auto database = db_client[db_name_];
        auto db_stats_value = database.run_command(bsoncxx::builder::stream::document{}
                                                   << "dbStats" << 1
                                                   << bsoncxx::builder::stream::finalize);
        auto db_stats = db_stats_value.view();
        unsigned int file_size_bytes = db_stats["fileSize"].get_int32();
        unsigned int file_size_mb = file_size_bytes / (1024 * 1024);
        return file_size_mb;
    }

    std::string MongodbLogger::getLatestDatabaseName()
    {
        mongocxx::client db_client{mongocxx::uri{}};
        auto cursor = db_client.list_databases();
        std::vector<std::string> all_db_names;
        for (auto db : cursor)
        {
            // find all dbs that start with db_name_prefix_
            std::string listed_db_name = db["name"].get_utf8().value.to_string();
            if (listed_db_name.rfind(db_name_prefix_, 0) == 0 && listed_db_name.size() > db_name_prefix_.size())
            {
                all_db_names.push_back(listed_db_name);
            }
        }

        if (!all_db_names.empty())
        {
            std::sort(all_db_names.begin(), all_db_names.end());
            return all_db_names.back();
        }
        else // if we don't find any databases with prefix, create a new one
        {
            return createNewDatabaseName();
        }
    }

    std::string MongodbLogger::createNewDatabaseName()
    {
        // generate a string of type "prefix_YYYYMMDDHHMMSS"
        std::chrono::time_point<std::chrono::system_clock> time_now = std::chrono::system_clock::now();
        std::time_t time_now_t = std::chrono::system_clock::to_time_t(time_now);
        std::tm now_tm = *std::localtime(&time_now_t);
        char buf[512];
        std::strftime(buf, 512, "%Y%m%d%H%M%S", &now_tm);
        return db_name_prefix_ + "_" + std::string(buf);
    }

    void MongodbLogger::setCurrentDb()
    {
        if (split_database_)
        {
            if (getDatabaseSize() > max_database_size_)
            {
                db_name_ = createNewDatabaseName();
                std::cout << "Maximum database size exceeded. Creating DB: " << db_name_ << std::endl;
            }
        }
    }

    /**
     * Returns a list of the names of all variables
     * corresponding to the given data source
     *
     * @param data_source_name name of a data source
     */
    std::vector<std::string> MongodbLogger::getVariables(std::string data_source_name)
    {
        mongocxx::client db_client{mongocxx::uri{}};
        auto database = db_client[db_name_];
        auto collections = database.list_collections();

        std::vector<std::string> variable_names;
        for (auto collection_info : collections)
        {
            std::string collection_name = collection_info["name"].get_utf8().value.to_string();
            if (collection_name.find(data_source_name) != std::string::npos)
            {
                auto collection = database[collection_name];
                auto doc = collection.find_one(bsoncxx::builder::stream::document{}
                                               << bsoncxx::builder::stream::finalize);

                auto document_view = (*doc).view();
                for (auto key : document_view)
                {
                    std::string variable_name = key.key().to_string();
                    if ((variable_name != "_id") && (variable_name != "timestamp"))
                    {
                        std::string variable_full_name = collection_name + "/" + variable_name;
                        variable_names.push_back(variable_full_name);
                    }
                }
            }
        }

        return variable_names;
    }

    /**
     * Returns a data dictionary in which each key represents a variable
     * and the value represents data corresponding to that variable
     * in a given time intervaldb
     *bsoncxx::builder::stream::finalize
     * @param variables a list of variable names
     * @param start_time a string representing the start data timestamp
     * @param end_time a string representing the end data timestamp
     */
    std::map<std::string, std::vector<std::string>> MongodbLogger::getData(const std::map<std::string, std::vector<std::string>> &variables,
                                                                           const std::string start_time,
                                                                           const std::string end_time)
    {
        mongocxx::client db_client{mongocxx::uri{}};
        std::map<std::string, std::vector<std::string>> data;
        for (auto group : variables)
        {
            std::string collection_name = group.first;
            auto collection = db_client[db_name_][collection_name];
            bsoncxx::builder::stream::document document{};
            auto cursor = collection.find(document << "timestamp" << bsoncxx::builder::stream::open_document
                                                   << "$gte" << std::stod(start_time)
                                                   << "$lte" << std::stod(end_time)
                                                   << bsoncxx::builder::stream::close_document
                                                   << bsoncxx::builder::stream::finalize);
            for(auto doc : cursor)
            {
                for (std::string var : group.second)
                {
                    std::string variable = collection_name + "/" + var;
                    if (data.count(variable) == 0)
                    {
                        data[variable] = std::vector<std::string>();
                    }
                    double timestamp = doc["timestamp"].get_double();
                    double value = doc[var].get_double();
                    data[variable].push_back("[" + std::to_string(timestamp) + ", " + std::to_string(value) + "]");
                }
            }
        }
        return data;
    }

    /**
     * Logs a list of integers
     *
     * @param variable name of the variable to log
     * @param timestamp data timestamp
     * @param values a list of integers that should be logged
     */
    void MongodbLogger::writeLog(std::string variable, double timestamp, const std::vector<int> &values)
    {
        setCurrentDb();
        mongocxx::client connection_{mongocxx::uri{}};
        std::string collection_name = variable;
        auto collection = connection_[db_name_][collection_name];
        bsoncxx::builder::stream::document document{};
        document << "timestamp" << timestamp;
        for (size_t i=0; i<values.size(); i++)
        {
            document << variable_names_[variable][i] << values[i];
        }
        collection.insert_one(document.view());
    }

    /**
     * Logs a list of floating-point values
     *
     * @param variable name of the variable to log
     * @param timestamp data timestamp
     * @param values a list of floats that should be logged
     */
    void MongodbLogger::writeLog(std::string variable, double timestamp, const std::vector<float> &values)
    {
        setCurrentDb();
        mongocxx::client connection_{mongocxx::uri{}};
        std::string collection_name = variable;
        auto collection = connection_[db_name_][collection_name];
        bsoncxx::builder::stream::document document{};
        document << "timestamp" << timestamp;
        for (size_t i=0; i<values.size(); i++)
        {
            document << variable_names_[variable][i] << values[i];
        }
        collection.insert_one(document.view());
    }

    /**
     * Logs a list of doubles
     *
     * @param variable name of the variable to log
     * @param timestamp data timestamp
     * @param values a list of doubles that should be logged
     */
    void MongodbLogger::writeLog(std::string variable, double timestamp, const std::vector<double> &values)
    {
        setCurrentDb();
        mongocxx::client connection_{mongocxx::uri{}};
        std::string collection_name = variable;
        auto collection = connection_[db_name_][collection_name];
        bsoncxx::builder::stream::document document{};
        document << "timestamp" << timestamp;
        for (size_t i=0; i<values.size(); i++)
        {
            document << variable_names_[variable][i] << values[i];
        }
        collection.insert_one(document.view());
    }


    /**
     * Logs a list of Boolean values
     *
     * @param variable name of the variable to log
     * @param timestamp data timestamp
     * @param values a list of booleans that should be logged
     */
    void MongodbLogger::writeLog(std::string variable, double timestamp, const std::vector<bool> &values)
    {
        setCurrentDb();
        mongocxx::client connection_{mongocxx::uri{}};
        std::string collection_name = variable;
        auto collection = connection_[db_name_][collection_name];
        bsoncxx::builder::stream::document document{};
        document << "timestamp" << timestamp;
        for (size_t i=0; i<values.size(); i++)
        {
            document << variable_names_[variable][i] << values[i];
        }
        collection.insert_one(document.view());
    }

    /**
     * Logs a list of strings
     *
     * @param variable name of the variable to log
     * @param timestamp data timestamp
     * @param values a list of strings that should be logged
     */
    void MongodbLogger::writeLog(std::string variable, double timestamp, const std::vector<std::string> &values)
    {
        setCurrentDb();
        mongocxx::client connection_{mongocxx::uri{}};
        std::string collection_name = variable;
        auto collection = connection_[db_name_][collection_name];
        bsoncxx::builder::stream::document document{};
        document << "timestamp" << timestamp;
        for (size_t i=0; i<values.size(); i++)
        {
            document << variable_names_[variable][i] << values[i];
        }
        collection.insert_one(document.view());
    }

    /**
     * Logs a two-dimensional list of floating-point values
     *
     * @param variable name of the variable to log
     * @param timestamp data timestamp
     * @param values a list of integers that should be logged
     */
    void MongodbLogger::writeLog(std::string variable, double timestamp, const std::vector<std::vector<float>> &values)
    {
        setCurrentDb();
        mongocxx::client connection_{mongocxx::uri{}};
        std::string collection_name = variable;
        auto collection = connection_[db_name_][collection_name];
        bsoncxx::builder::stream::document document{};
        document << "timestamp" << timestamp;

        size_t var_idx = 0;
        for (size_t i=0; i<values.size(); i++)
        {
            if (boost::starts_with(variable_names_[variable][var_idx], "--"))
            {
                auto array_builder = document << variable_names_[variable][var_idx].substr(2, variable_names_[variable][var_idx].length() - 2) << bsoncxx::builder::stream::open_array;
                for (int j=0; j<values[i].size(); j++)
                {
                    array_builder << values[i][j];
                }
                array_builder << bsoncxx::builder::stream::close_array;
                var_idx++;
            }
            else
            {
                for (int j=0; j<values[i].size(); j++)
                {
                    document << variable_names_[variable][var_idx] << values[i][j];
                    var_idx++;
                }
            }
        }
        collection.insert_one(document.view());
    }

    /**
     * Logs a three-dimensional list of floating-point values
     *
     * @param variable name of the variable to log
     * @param timestamp data timestamp
     * @param values a list of three dimensional floats that should be logged
     */
    void MongodbLogger::writeLog(std::string variable, double timestamp, const std::vector<std::vector<std::vector<float>>> &values)
    {
        setCurrentDb();
        mongocxx::client connection_{mongocxx::uri{}};
        std::string collection_name = variable;
        auto collection = connection_[db_name_][collection_name];
        bsoncxx::builder::stream::document document{};
        document << "timestamp" << timestamp;

        size_t var_idx = 0;
        for (size_t i=0; i<values.size(); i++)
        {
            if (boost::starts_with(variable_names_[variable][var_idx], "~~"))
            {
                auto array_builder = document << variable_names_[variable][var_idx].substr(2, variable_names_[variable][var_idx].length() - 2) << bsoncxx::builder::stream::open_array;
                const std::vector<std::vector<float>> inner_values = values[i];
                for (int j = 0; j < inner_values.size(); j++)
                {
                    array_builder << bsoncxx::builder::stream::open_array;
                    for (int k = 0; k < inner_values[j].size(); k++)
                    {
                        array_builder << inner_values[j][k];
                    }
                    array_builder << bsoncxx::builder::stream::close_array;
                }
                array_builder << bsoncxx::builder::stream::close_array;
                var_idx++;
            }
            else
            {
                for (int j=0; j<values[i].size(); j++)
                {
                    for (int k = 0; k < values[i][j].size(); k++)
                    {
                        document << variable_names_[variable][var_idx] << values[i][j][k];
                        var_idx++;
                    }
                }
            }
        }
        collection.insert_one(document.view());
    }

    /**
     * Logs a list of floats and a list of strings
     *
     * @param variable name of the variable to log
     * @param timestamp data timestamp
     * @param values a list of floats  that should be logged
     * @param str_values a list of strings that should be logged
     */
    void MongodbLogger::writeLog(std::string variable, double timestamp,
            const std::vector<float> &values, const std::vector<std::string> &str_values)
    {
        setCurrentDb();
        mongocxx::client connection_{mongocxx::uri{}};
        std::string collection_name = variable;
        auto collection = connection_[db_name_][collection_name];
        bsoncxx::builder::stream::document document{};
        document << "timestamp" << timestamp;
        int var_idx = 0;
        for (size_t i=0; i<values.size(); i++)
        {
            document << variable_names_[variable][var_idx] << values[i];
            var_idx++;
        }
        for (size_t i=0; i<str_values.size(); i++)
        {
            document << variable_names_[variable][var_idx] << str_values[i];
            var_idx++;
        }
        collection.insert_one(document.view());
    }

    /**
     * Logs a json string
     *
     * @param variable name of the variable to log
     * @param timestamp data timestamp
     * @param json_string a string that should be logged
     */
    void MongodbLogger::writeLog(std::string variable, double timestamp, const std::string &json_string)
    {
        setCurrentDb();
        mongocxx::client connection_{mongocxx::uri{}};
        std::string collection_name = variable;
        auto collection = connection_[db_name_][collection_name];
        bsoncxx::builder::stream::document document{};
        document << "timestamp" << timestamp;
        bsoncxx::document::value value = bsoncxx::from_json(json_string);
        document << bsoncxx::builder::concatenate(value.view());
        collection.insert_one(document.view());
    }

    /**
     * Logs a json string
     *
     * @param variable name of the variable to log
     * @param json_string a string in json format that should be logged; the timestamp is part of the message
     */
    void MongodbLogger::writeLog(std::string variable, const std::string &str)
    {
        setCurrentDb();
        mongocxx::client connection_{mongocxx::uri{}};
        std::string collection_name = variable;
        auto collection = connection_[db_name_][collection_name];
        bsoncxx::builder::stream::document document{};
        bsoncxx::document::value value = bsoncxx::from_json(str);
        document << bsoncxx::builder::concatenate(value.view());
        collection.insert_one(document.view());
    }
}
