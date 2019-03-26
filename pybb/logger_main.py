#!/usr/bin/env python3
import sys

from black_box.config.config_file_reader import ConfigFileReader
from black_box.datalogger.loggers.mongodb_logger import MongoDBLogger
from black_box.datalogger.data_readers.rostopic_reader import ROSTopicReader
from black_box.datalogger.data_readers.zyre_reader import ZyreReader
from black_box.datalogger.data_readers.json_zmq_reader import JsonZmqReader

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: logger_main.py [absolute-path-to-black-box-config-file]')
        sys.exit(1)
    bb_config_file = sys.argv[1]

    debug = False
    if '--debug' in sys.argv:
        debug = True

    config_params = ConfigFileReader.load_config(bb_config_file)
    if debug:
        print(config_params)

    logger = MongoDBLogger(db_name=config_params.default.db_name, db_port=27017,
                           split_db=config_params.default.split_db,
                           max_db_size=config_params.default.max_db_size)
    logger.write_metadata(config_params)

    zyre_reader = None
    if config_params.zyre:
        zyre_reader = ZyreReader(config_params.zyre, logger)

    json_zmq_reader = None
    if config_params.zmq:
        json_zmq_reader = JsonZmqReader(config_params.zmq.url,
                                        config_params.zmq.port,
                                        config_params.zmq.topics,
                                        logger)

    rostopic_reader = None
    if config_params.ros:
        rostopic_reader = ROSTopicReader('rostopic_reader',
                                         config_params.ros,
                                         config_params.default.max_frequency,
                                         logger)

    try:
        # this output is before starting the ROS topic reader
        # because that one will block the execution
        print('[{0}] Logger configured; ready to log data'.format(config_params.zyre.node_name))

        if json_zmq_reader:
            json_zmq_reader.start()

        if rostopic_reader:
            rostopic_reader.start()
    except (KeyboardInterrupt, SystemExit):
        if zyre_reader:
            zyre_reader.shutdown()

        if rostopic_reader:
            rostopic_reader.stop()
