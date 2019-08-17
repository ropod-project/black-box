import pymongo as pm
from black_box.datalogger.loggers.logger_base import LoggerBase
from black_box.config.config_utils import ConfigUtils
from black_box_tools.db_utils import DBUtils

class MongoDBLogger(LoggerBase):
    def __init__(self, db_name, db_port, split_db, max_db_size):
        self.db_name = db_name
        self.db_port = db_port
        self.split_db = split_db
        self.max_db_size = max_db_size

    def write_metadata(self, config_params):
        '''If a "black_box_metadata" collection doesn't exist in the black box database,
        creates the collection and stores the metadata for all logged variables there.

        @param config_params -- a black_box.config.config_params.ConfigParams instance

        '''
        if config_params:
            (host, port) = DBUtils.get_db_host_and_port()
            if port != self.db_port:
                port = self.db_port

            client = pm.MongoClient(host=host, port=port)
            db = client[self.db_name]
            collection_names = db.list_collection_names()
            if 'black_box_metadata' in collection_names:
                print('[write_metadata] {0} already has a "black_box_metadata" collection'.format(self.db_name))
            else:
                print('[write_metadata] Saving metadata')
                collection = db['black_box_metadata']
                if config_params.ros:
                    for topic_params in config_params.ros.topic:
                        collection_name = ConfigUtils.get_full_variable_name('ros', topic_params.name)
                        if topic_params.metadata:
                            metadata = {}
                            metadata['collection_name'] = collection_name
                            metadata['ros'] = {}
                            metadata['ros']['topic_name'] = topic_params.metadata.topic_name
                            metadata['ros']['msg_type'] = topic_params.metadata.msg_type
                            metadata['ros']['direct_msg_mapping'] = topic_params.metadata.direct_msg_mapping
                            collection.insert_one(metadata)

                if config_params.zmq:
                    for topic_params in config_params.zmq.topics:
                        collection_name = ConfigUtils.get_full_variable_name('zmq', topic_params.name)
                        if topic_params.metadata:
                            metadata = {}
                            metadata['collection_name'] = collection_name
                            metadata['ros'] = {}
                            metadata['ros']['topic_name'] = topic_params.metadata.topic_name
                            metadata['ros']['msg_type'] = topic_params.metadata.msg_type
                            metadata['ros']['direct_msg_mapping'] = topic_params.metadata.direct_msg_mapping
                            collection.insert_one(metadata)
        else:
            raise AssertionError('[write_metadata] config_params needs to be of type ' + \
                                 'black_box.config.config_params.ConfigParams')

    def log_data(self, variable, timestamp, data):
        '''Logs the data dictionary for the given variable;
        adds the timestamp to the data dictionary before logging.

        @param variable -- name of a variable to be logged
        @param timestamp -- a timestamp (epoch in seconds)
        @param data -- a dictionary to be logged

        '''
        (host, port) = DBUtils.get_db_host_and_port()
        if port != self.db_port:
            port = self.db_port

        client = pm.MongoClient(host=host, port=port)
        db = client[self.db_name]
        collection = db[variable]
        data['timestamp'] = timestamp
        collection.insert_one(data)
