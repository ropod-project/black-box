#ifndef QUERY_INTERFACE_FACTORY_H
#define QUERY_INTERFACE_FACTORY_H

#include <memory>
#include <string>

#include "query_interface_base.h"
#include "ethercat_query_interface.h"
#include "ros_query_interface.h"
#include "zmq_query_interface.h"
#include "zyre_query_interface.h"
#include "config/config_enums.hpp"

namespace data_retrieval
{
    /**
     * A factory for creating query interface instances
     *
     * @author Alex Mitrevski, Santosh Thoduka
     * @contact aleksandar.mitrevski@h-brs.de, santosh.thoduka@h-brs.de
     */
    class QueryInterfaceFactory
    {
    public:
        static std::shared_ptr<QueryInterfaceBase> getQueryInterface(std::string data_source);
    };
}

#endif
