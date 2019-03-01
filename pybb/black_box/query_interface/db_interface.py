import pymongo as pm
from black_box_tools.data_utils import DataUtils
from black_box_tools.db_utils import DBUtils

class DBInterface(object):
    '''An interface to a black box database

    @author Alex Mitrevski
    @contact aleksandar.mitrevski@h-brs.de

    '''
    def __init__(self, db_name='logs', port=27017):
        self.db_name = db_name
        self.db_port = port

    def get_variables(self, data_source):
        '''Returns a list of all stored variables corresponding to the input data source

        Keyword arguments:
        @param data_source -- a string denoting the name of a data source; collection names
                              corresponding to data from this data source are expected to
                              start with the data source name

        '''
        client = pm.MongoClient(port=self.db_port)
        database = client[self.db_name]
        collection_names = database.list_collection_names()

        variable_list = []
        for collection_name in collection_names:
            if collection_name == 'system.indexes' or \
               not collection_name.startswith(data_source):
                continue

            collection = database[collection_name]
            variable_names = DataUtils.get_variable_list(collection_name, collection)
            variable_list.extend(variable_names)
        return variable_list

    def get_data(self, collection_name, variable_names, start_time=-1, end_time=-1):
        '''Returns a dictionary in which each key is a full variable name
        (namely a variable name of the format "collection_name/variable_name",
        where "variable_name" is a flattened version of a variable stored in the collection)
        and each value is a list of "[timestamp, value]" strings, namely
        each entry in the list corresponds to a value of the variable at
        a particular timestamp (the entries are in string format to allow "value"
        to be of different types)

        Keyword arguments:
        @param collection_name -- name corresponding to a collection from the log database
        @param variable_names -- list of variable names that should be retrieved from the collection
        @param start_time -- a UNIX timestamp in seconds representing the start time
                             for the queried data (default -1, in which case
                             there is no lower bound for the timestamp
                             of the retrieved data)
        @param end_time -- a UNIX timestamp in seconds representing the end time
                           for the queried data (default -1, in which case there is
                           no upper bound for the timestamp of the retrieved data)

        '''
        docs = DBUtils.get_doc_cursor(
                self.db_name, collection_name, start_time, end_time, self.db_port)

        var_data = {}
        var_full_names = {}
        for var_name in variable_names:
            full_var_name = '{0}/{1}'.format(collection_name, var_name)
            var_data[full_var_name] = []
            var_full_names[var_name] = full_var_name

        for doc in docs:
            for var_name in variable_names:
                var_value = DataUtils.get_var_value(doc, var_name)
                var_data[var_full_names[var_name]].append('[{0}, {1}]'.format(doc['timestamp'],
                                                                              var_value))
        return var_data

    def get_latest_data(self, collection_name, variable_names):
        '''Returns a dictionary in which each key is a full variable name
        (namely a variable name of the format "collection_name/variable_name",
        where "variable_name" is a flattened version of a variable stored in the collection)
        and the value is a list string "[timestamp, value]", namely the latest value
        of the variable together with its timestamp (the list is a string to allow "value"
        to be of different types). If the given collection does not exist or
        there are no documents in it, returns an empty dictionary.

        Keyword arguments:
        @param collection_name -- name corresponding to a collection from the log database
        @param variable_names -- list of variable names that should be retrieved from the collection

        '''
        client = pm.MongoClient(port=self.db_port)
        database = client[self.db_name]
        collection = database[collection_name]
        doc = collection.find_one(sort=[('timestamp', pm.DESCENDING)])

        var_data = {}
        var_full_names = {}
        for var_name in variable_names:
            full_var_name = '{0}/{1}'.format(collection_name, var_name)
            var_data[full_var_name] = None
            var_full_names[var_name] = full_var_name

        if doc:
            for var_name in variable_names:
                var_value = DataUtils.get_var_value(doc, var_name)
                var_data[var_full_names[var_name]] = '[{0}, {1}]'.format(doc['timestamp'],
                                                                         var_value)
        return var_data
