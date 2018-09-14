#include "query_interface_factory.h"

namespace data_retrieval
{
    /**
     * Returns a shared QueryInterfaceBase pointer to a query interface
     * instance for the given data source
     *
     * @param data_source the name of a data source
     */
    std::shared_ptr<QueryInterfaceBase> QueryInterfaceFactory::getQueryInterface(std::string data_source)
    {
        if (data_source == config::DataSourceNames::ETHERCAT)
        {
            std::shared_ptr<EthercatQueryInterface> interface(new EthercatQueryInterface());
            return interface;
        }
        else if (data_source == config::DataSourceNames::ROS)
        {
            std::shared_ptr<RosQueryInterface> interface(new RosQueryInterface());
            return interface;
        }
        else if (data_source == config::DataSourceNames::ZMQ)
        {
            std::shared_ptr<ZmqQueryInterface> interface(new ZmqQueryInterface());
            return interface;
        }
        else if (data_source == config::DataSourceNames::ZYRE)
        {
            std::shared_ptr<ZyreQueryInterface> interface(new ZyreQueryInterface());
            return interface;
        }
    }
}
