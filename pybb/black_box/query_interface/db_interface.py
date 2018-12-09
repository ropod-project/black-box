import pymongo as pm

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
            variable_names = self.__get_variable_list(collection_name, collection)
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
        client = pm.MongoClient(port=self.db_port)
        database = client[self.db_name]
        collection = database[collection_name]

        docs = {}
        if start_time == -1 and end_time == -1:
            docs = collection.find({})
        elif start_time == -1:
            docs = collection.find({'timestamp': {'$lte': end_time}})
        elif end_time == -1:
            docs = collection.find({'timestamp': {'$gte': start_time}})
        else:
            docs = collection.find({'timestamp': {'$gte': start_time, '$lte': end_time}})

        var_data = {}
        var_full_names = {}
        for var_name in variable_names:
            full_var_name = '{0}/{1}'.format(collection_name, var_name)
            var_data[full_var_name] = []
            var_full_names[var_name] = full_var_name

        for doc in docs:
            for var_name in variable_names:
                var_value = self.__get_var_value(doc, var_name)
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
                var_value = self.__get_var_value(doc, var_name)
                var_data[var_full_names[var_name]] = '[{0}, {1}]'.format(doc['timestamp'],
                                                                         var_value)
        return var_data

    def __get_variable_list(self, collection_name, collection):
        '''Returns a list of the names of all variables stored in the input collection

        Keyword arguments:
        @param collection -- a MongoDB collection

        '''
        doc = collection.find_one({})
        variables = self.__get_flattened_variable_names(doc, collection_name)
        return variables

    def __get_flattened_variable_names(self, current_dict, current_var_name):
        '''Recursive method that returns a flattened list of the variable names
        in the given dictionary, where variables at different levels are separated by a /.

        @param current_dict -- current variable dictionary (initially a full MongoDB document;
                               the value gets updated recursively)
        @param current_var_name -- name of the variable at the current level of recursion
                                   (initially set to the name of a collection so that the full
                                   variable names are of the form "collection_name/variable_name")

        Example:
        If "current_dict" is initially given as
        {
            linear:
            {
                x: -1,
                y: -1,
                z: -1
            },
            angular:
            {
                x: -1,
                y: -1,
                z: -1
            }
        }
        and "current_var_name" is initially "ros_cmd_vel", the returned list will contain
        the following variable names:
        ["ros_cmd_vel/linear/x", "ros_cmd_vel/linear/y", "ros_cmd_vel/linear/z",
         "ros_cmd_vel/angular/x", "ros_cmd_vel/angular/y", "ros_cmd_vel/angular/z"]

        If the input dictionary contains a list of items, the items will also
        be separated by a /. This is best illustrated by an example. If "current_dict"
        is given as
        {
            commands:
            [
                {
                    setpoint1: -1,
                    setpoint2: -1
                },
                {
                    setpoint1: -1,
                    setpoint2: -1
                }
            ],
            sensors:
            [
                {
                    velocity1: -1,
                    velocity2: -1
                },
                {
                    velocity1: -1,
                    velocity2: -1
                }
            ]
        }
        and "current_var_name" is "ros_sw_data", the returned list will contain
        the following variable names:
        ["ros_sw_data/commands/0/setpoint1", "ros_sw_data/commands/0/setpoint2",
         "ros_sw_data/commands/1/setpoint1", "ros_sw_data/commands/1/setpoint2",
         "ros_sw_data/sensors/0/velocity1", "ros_sw_data/sensors/0/velocity2",
         "ros_sw_data/sensors/1/velocity1", "ros_sw_data/sensors/1/velocity2"]

        '''
        if not isinstance(current_dict, dict):
            return [current_var_name]

        variables = []
        for name in current_dict:
            if name == '_id' or name == 'timestamp':
                continue

            var_list = []
            if not isinstance(current_dict[name], list):
                new_var_name = '{0}/{1}'.format(current_var_name, name)
                var_list = self.__get_flattened_variable_names(current_dict[name],
                                                               new_var_name)
                variables.extend(var_list)
            else:
                new_var_name = current_var_name
                for i, list_item in enumerate(current_dict[name]):
                    new_var_name = '{0}/{1}/{2}'.format(current_var_name, name, str(i))
                    var_list = self.__get_flattened_variable_names(list_item, new_var_name)
                    variables.extend(var_list)
        return variables

    def __get_var_value(self, item_dict, var_name):
        '''Returns the value of "var_name" in the given dictionary; returns None
        if the variable does not exit in the dictionary. "var_name" is expected
        to be a flattened variable name with items at different levels
        separated by a / (the name should however NOT contain the name of the
        collection as returned by self.__get_flattened_variable_names, but only
        the names of the items that are included in the document).

        @param item_dict -- a MongoDB document
        @param var_name -- name of the variable whose value is queried

        Example:
        If "item_dict" is given as
        {
            linear:
            {
                x: 1,
                y: 2,
                z: 2
            }
        }
        and "var_name" is "linear/x", the returned value will be 1.

        If "item_dict" has a list of items, the items should also be separated by
        a / (as done by self.__get_flattened_variable_names). For instance, if
        "item_dict" is given as
        {
            sensors:
            [
                {
                    velocity1: 0.1,
                    velocity2: 0.2
                },
                {
                    velocity1: 0.0,
                    velocity2: 0.1
                }
            ]
        }
        and we want to get the value of the second "velocity2" item,
        "var_name" should be given as "sensors/1/velocity2".

        '''
        current_item = item_dict
        separator_idx = var_name.find('/')
        current_var_name = var_name
        while separator_idx != -1:
            current_var_name = var_name[0:separator_idx]
            if current_var_name.isdigit():
                current_var_name = int(current_var_name)
            current_item = current_item[current_var_name]
            var_name = var_name[separator_idx+1:]
            separator_idx = var_name.find('/')

        val = None
        try:
            if var_name.isdigit():
                var_name = int(var_name)
            val = current_item[var_name]
        except KeyError:
            print('An unknown variable {0} was requested; returning None'.format(var_name))
        return val
