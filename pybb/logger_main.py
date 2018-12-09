import sys

from black_box.config.config_file_reader import ConfigFileReader
from black_box.datalogger.loggers.mongodb_logger import MongoDBLogger
from black_box.datalogger.data_readers.rostopic_reader import ROSTopicReader

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: logger_main.py [absolute-path-to-black-box-config-file]')
    bb_config_file = sys.argv[1]

    debug = False
    if '--debug' in sys.argv:
        debug = True

    config_params = ConfigFileReader.load_config(bb_config_file)
    if debug:
        print(config_params)

    logger = MongoDBLogger(db_name='log_test', db_port=27017,
                           split_db=config_params.default.split_db,
                           max_db_size=config_params.default.max_db_size)

    rostopic_reader = ROSTopicReader('rostopic_reader',
                                     config_params.ros,
                                     config_params.default.max_frequency,
                                     logger)

    # this last output is before starting the ROS topic reader
    # because that one will block the execution
    print('[{0}] Logger configured; ready to log data'.format(config_params.zyre.node_name))
    rostopic_reader.start()
