#!/usr/bin/env python3
import sys
import time

from black_box.config.config_file_reader import ConfigFileReader
from black_box.datalogger.loggers.mongodb_logger import MongoDBLogger
from black_box.datalogger.pyre_comm.bb_pyre_comm import BlackBoxPyreCommunicator

from black_box.datalogger.data_readers.rostopic_reader import ROSTopicReader
from black_box.datalogger.data_readers.zyre_reader import ZyreReader
from black_box.datalogger.data_readers.json_zmq_reader import JsonZmqReader
from black_box.datalogger.data_readers.event_reader import EventReader

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
    
    readers = {}
    for reader_name in config_params.__dict__.keys():
        if reader_name != 'default': # TODO:need better way to ignore default
            readers[reader_name] = None
    print(readers)


    if config_params.zyre:
        readers['zyre'] = ZyreReader(config_params.zyre, logger)

    if config_params.zmq:
        readers['zmq'] = JsonZmqReader(config_params.zmq.url,
                                        config_params.zmq.port,
                                        config_params.zmq.topics,
                                        logger)

    if config_params.ros:
        readers['ros'] = ROSTopicReader('rostopic_reader',
                                         config_params.ros,
                                         config_params.default.max_frequency,
                                         logger)

    if config_params.event:
        readers['event'] = EventReader(config_params.event,
                                     config_params.default.max_frequency,
                                     logger)

    bb_pyre_comm = BlackBoxPyreCommunicator(['ROPOD'], config_params.zyre.node_name)

    try:

        for reader_name in readers:
            if readers[reader_name]:
                readers[reader_name].start_logging()

        print('[{0}] Logger configured; ready to log data'.format(config_params.zyre.node_name))
        logging = True
        while True:
            if logging != bb_pyre_comm.logging:
                if bb_pyre_comm.logging:
                    for reader_name in readers:
                        if readers[reader_name]:
                            readers[reader_name].start_logging()
                    print('[{0}] Started logging'.format(config_params.zyre.node_name))
                else:
                    for reader_name in readers:
                        if readers[reader_name]:
                            readers[reader_name].stop_logging()
                    print('[{0}] Stopped logging'.format(config_params.zyre.node_name))
                logging = bb_pyre_comm.logging

            time.sleep(0.1)
    except (KeyboardInterrupt, SystemExit):
        print('[logger_main] Interrupted. Exiting...')
        for reader_name in readers:
            if readers[reader_name]:
                readers[reader_name].stop_logging()
        if readers['zyre']:
            readers['zyre'].shutdown()

        bb_pyre_comm.shutdown()
