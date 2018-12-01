#ifndef ROS_QUERY_INTERFACE_H
#define ROS_QUERY_INTERFACE_H

#include "query_interface_base.h"

namespace data_retrieval
{
    /**
     * An interface for answering ROS data queries
     *
     * @author Alex Mitrevski, Santosh Thoduka
     * @contact aleksandar.mitrevski@h-brs.de, santosh.thoduka@h-brs.de
     */
    class RosQueryInterface : public QueryInterfaceBase
    {
    public:
        /**
         * Returns a list of the names of all ROS variables
         *
         * @param data_logger a DataLogger shared pointer
         */
        virtual std::vector<std::string> getVariables(const std::shared_ptr<loggers::DataLogger> data_logger);

        /**
         * Returns a data dictionary in which each key represents a ROS variable
         * and the value represents data corresponding to that variable
         * in a given time interval
         *
         * @param data_logger a DataLogger shared pointer
         * @param variables a list of variable names
         * @param start_time a string representing the start data timestamp
         * @param end_time a string representing the end data timestamp
         */
        virtual std::map<std::string, std::vector<std::string>> getData(const std::shared_ptr<loggers::DataLogger> data_logger,
                                                                        const std::map<std::string, std::vector<std::string>> &variables,
                                                                        const std::string start_time,
                                                                        const std::string end_time);
    };
}

#endif
