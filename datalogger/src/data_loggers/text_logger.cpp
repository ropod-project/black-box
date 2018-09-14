#include "datalogger/data_loggers/text_logger.hpp"

namespace loggers
{
    /**
     * @param group_names a list of variable group names
     * @param variable_names a two-dimensional list of variable names corresponding to the group names
     */
    TextLogger::TextLogger(const std::vector<std::string> &group_names, const std::map<std::string, std::vector<std::string>> &variable_names)
    {
        group_names_ = group_names;
        variable_names_ = variable_names;

        for (unsigned int i=0; i<group_names.size(); i++)
        {
            file_writers_.insert(std::make_pair(group_names[i], std::make_shared<std::ofstream>(getFileName(group_names[i]))));
        }
    }

    TextLogger::~TextLogger()
    {
        std::map<std::string, std::shared_ptr<std::ofstream> >::iterator iter = file_writers_.begin();
        for (; iter != file_writers_.end(); ++iter)
        {
            iter->second.get()->close();
        }
        file_writers_.clear();
    }

    /**
     * Returns a list of the names of all variables
     * corresponding to the given data source
     *
     * @param data_source_name name of a data source
     */
    std::vector<std::string> TextLogger::getVariables(std::string data_source_name)
    {
        return std::vector<std::string>();
    }

    /**
     * Returns a data dictionary in which each key represents a variable
     * and the value represents data corresponding to that variable
     * in a given time interval
     *
     * @param variables a list of variable names
     * @param start_time a string representing the start data timestamp
     * @param end_time a string representing the end data timestamp
     */
    std::map<std::string, std::vector<std::string>> TextLogger::getData(const std::map<std::string, std::vector<std::string>> &variables,
                                                                        const std::string start_time,
                                                                        const std::string end_time)
    {
        return std::map<std::string, std::vector<std::string>>();
    }


    // bool TextLogger::addVariable(const std::string &variable_name)
    // {
    //     if (file_writers_.find(variable_name) == file_writers_.end())
    //     {
    //         file_writers_.insert(std::make_pair(variable_name, std::make_shared<std::ofstream>(getFileName(variable_name))));
    //         group_names_.push_back(variable_name);
    //         return true;
    //     }
    //     else
    //     {
    //         // variable already exists
    //         return false;
    //     }
    // }

    /**
     * Logs a list of integers
     *
     * @param variable name of the variable to log
     * @param timestamp data timestamp
     * @param values a list of integers that should be logged
     */
    void TextLogger::writeLog(std::string variable, double timestamp, const std::vector<int> &values)
    {
        std::shared_ptr<std::ofstream> fout = file_writers_[variable];
        *fout << std::fixed << timestamp;
        for (int i = 0; i < values.size(); i++)
        {
            *fout << ", " << values[i];
        }
        *fout << std::endl;
    }

    /**
     * Logs a list of floating-point values
     *
     * @param variable name of the variable to log
     * @param timestamp data timestamp
     * @param values a list of floats that should be logged
     */
    void TextLogger::writeLog(std::string variable, double timestamp, const std::vector<float> &values)
    {
        std::shared_ptr<std::ofstream> fout = file_writers_[variable];
        *fout << std::fixed << timestamp;
        for (int i = 0; i < values.size(); i++)
        {
            *fout << ", " << values[i];
        }
        *fout << std::endl;
    }

    /**
     * Logs a list of doubles
     *
     * @param variable name of the variable to log
     * @param timestamp data timestamp
     * @param values a list of doubles that should be logged
     */
    void TextLogger::writeLog(std::string variable, double timestamp, const std::vector<double> &values)
    {
        std::shared_ptr<std::ofstream> fout = file_writers_[variable];
        *fout << std::fixed << timestamp;
        for (int i = 0; i < values.size(); i++)
        {
            *fout << ", " << values[i];
        }
        *fout << std::endl;
    }

    /**
     * Logs a list of Boolean values
     *
     * @param variable name of the variable to log
     * @param timestamp data timestamp
     * @param values a list of booleans that should be logged
     */
    void TextLogger::writeLog(std::string variable, double timestamp, const std::vector<bool> &values)
    {
        std::shared_ptr<std::ofstream> fout = file_writers_[variable];
        *fout << std::fixed << timestamp;
        for (int i = 0; i < values.size(); i++)
        {
            *fout << ", " << values[i];
        }
        *fout << std::endl;
    }

    /**
     * Logs a list of strings
     *
     * @param variable name of the variable to log
     * @param timestamp data timestamp
     * @param values a list of strings that should be logged
     */
    void TextLogger::writeLog(std::string variable, double timestamp, const std::vector<std::string> &values)
    {
        std::shared_ptr<std::ofstream> fout = file_writers_[variable];
        *fout << std::fixed << timestamp;
        for (int i = 0; i < values.size(); i++)
        {
            *fout << ", " << values[i];
        }
        *fout << std::endl;
    }

    /**
     * Logs a two-dimensional list of floating-point values
     *
     * @param variable name of the variable to log
     * @param timestamp data timestamp
     * @param values a list of integers that should be logged
     */
    void TextLogger::writeLog(std::string variable, double timestamp, const std::vector<std::vector<float>> &values)
    {
    }

    /**
     * Logs a json string
     *
     * @param variable name of the variable to log
     * @param timestamp data timestamp
     * @param json_string a string that should be logged
     */
    void TextLogger::writeLog(std::string variable, double timestamp, const std::string &json_string)
    {
        std::shared_ptr<std::ofstream> fout = file_writers_[variable];
        *fout << std::fixed << timestamp;
        *fout << json_string << std::endl;
    }

    /**
     * Returns the name of a txt file with the given variable name
     * (i.e. the name is just '<variable_name>.txt')
     *
     * @param variable name name of a variable
     */
    std::string TextLogger::getFileName(const std::string &variable_name)
    {
        return variable_name + ".txt";
    }
}
