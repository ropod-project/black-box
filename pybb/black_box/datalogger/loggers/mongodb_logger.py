from black_box.datalogger.loggers.logger_base import LoggerBase
import pymongo as pm

class MongoDBLogger(LoggerBase):
    def __init__(self, db_name, db_port, split_db, max_db_size):
        self.db_name = db_name
        self.db_port = db_port
        self.split_db = split_db
        self.max_db_size = max_db_size

    def log_data(self, variable, timestamp, data):
        client = pm.MongoClient(port=self.db_port)
        db = client[self.db_name]
        collection = db[variable]
        data['timestamp'] = timestamp
        collection.insert_one(data)
