#ifndef ZYRE_READER_H
#define ZYRE_READER_H

#include <string>
#include <vector>
#include <zyre.h>

#include "config/config_params.hpp"
#include "config/config_enums.hpp"
#include "utils/zyre/zyre_interface.h"
#include "utils/zyre/zyre_utils.h"
#include "datalogger/data_loggers/data_logger.hpp"
#include "datalogger/data_readers/data_reader.hpp"

namespace readers
{
    /**
     * An interface for receiving zyre data
     *
     * @author Alex Mitrevski, Santosh Thoduka
     * @contact aleksandar.mitrevski@h-brs.de, santosh.thoduka@h-brs.de
     */
    class ZyreReader : public utils::IZyreMsgReceive, public DataReader
    {
    public:
        /**
         * @param config_params zyre-specific configuration parameterss
         * @param data_logger a data logger instance for logging the received data
         */
        ZyreReader(const config::ZyreParams &config_params, std::shared_ptr<loggers::DataLogger> data_logger);
        virtual ~ZyreReader();

        /**
         * Adds the internal zyre node to all groups specified in the config parameters
         */
        void start() { }

        /**
         * Removes the internal zyre node from all groups specified in the config parameters
         */
        void stop() { }

        virtual void initialise() { }
        virtual void react(zmsg_t *msg);
        virtual void cleanup() { }
    private:
        zactor_t * actor_;
    };
}

#endif /* ZYRE_READER_H */
