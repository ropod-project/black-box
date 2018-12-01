#!/usr/bin/env python
from __future__ import print_function
import pymongo as pm

class BlackBoxODM(object):
    '''A module for generating Python structures from black box data
    stored in a MongoDB database.

    @author Alex Mitrevski
    @contact aleksandar.mitrevski@h-brs.de

    '''
    def __init__(self, db_name):
        # name of the black box database
        self.db_name = db_name

        # pymongo client
        self.__db_client = pm.MongoClient()

        # MongoDB database
        self.__database = self.__db_client[self.db_name]

    def deserialize_logs(self):
        '''Generates Python classes from the black box collections stored in
        the database "self.db_name"; the generated classes thus represent a model
        of the black box data. The classes for each collection are saved in a script
        named [collection_name].py.
        '''
        collection_names = self.__database.list_collection_names()
        try:
            collection_names.remove('system.indexes')
        except ValueError:
            pass

        for collection_name in collection_names:
            print('Generating Python class from collection "{0}"'.format(collection_name))
            collection = self.__database[collection_name]
            doc = collection.find_one()
            class_name = ''.join([x.title() for x in collection_name.split('_')])

            class_definitions = dict()
            class_definitions[class_name] = dict()

            variable_dict, variable_mapping_dict = self.__generate_variable_dicts(doc)
            self.__generate_class_definitions(self.db_name, collection_name,
                                              variable_dict, variable_mapping_dict,
                                              class_name, class_definitions)
            self.__generate_source_file(self.db_name, collection_name, class_definitions)
            print('Script "{0}.py" generated'.format(collection_name))

    def __generate_variable_dicts(self, doc):
        '''Returns two nested dictionaries:
        variable_dict a nested dictionary in which the keys represent variable or
                      object names; in the case of an object, the values represent
                      names of variables (or objects) belonging to the object; in
                      the case of a variable, the value is equivalent to the key
        variable_mapping_dict a nested dictionary in which the keys represent
                              variable or object names (same as in variable_dict);
                              in the case of an object, the values represent
                              names of variables (or objects) belonging to the object;
                              in the case of a variable, the value represents
                              the name of a MongoDB document value that correspond
                              to the variable

        Keyword argument:
        @param doc a MongoDB document

        Example: Consider the following document that might be stored in a database:
        {
            "_id" : ObjectId("5ba8b821805b917146371ed2"),
            "angular/x" : 0.0,
            "angular/y" : 0.0,
            "angular/z" : 0.0,
            "linear/x" : 1.0,
            "linear/y" : 0.0,
            "linear/z" : 0.0,
            "timestamp" : 1537783841.417
        }

        The two returned dictionaries will have the following form in this case:

        variable_dict:
        {
            'linear':
            {
                'y': 0.0,
                'z': 0.0,
                'x': 0.0
            },
            'angular':
            {
                'y': 0.0,
                'z': 0.0,
                'x': 0.0
            },
            '_id': ObjectId('5ba8b821805b917146371ed2'),
            'timestamp': 1537783841.417
        }

        variable_mapping_dict:
        {
            'linear':
            {
                'y': 'linear/y',
                'z': 'linear/z',
                'x': 'linear/x'
            },
            'angular':
            {
                'y': 'angular/y',
                'z': 'angular/z',
                'x': 'angular/x'
            },
            '_id': '_id',
            'timestamp': 'timestamp'
        }

        '''
        variable_dict = dict()
        variable_mapping_dict = dict()
        for full_variable_name in doc:
            slash_indices = [0]
            current_variable_dict = variable_dict
            current_variable_mapping_dict = variable_mapping_dict
            for i, char in enumerate(full_variable_name):
                if char == '/':
                    slash_indices.append(i+1)
                    name_component = full_variable_name[slash_indices[-2]:slash_indices[-1]-1]
                    if name_component not in current_variable_dict:
                        current_variable_dict[name_component] = dict()
                        current_variable_mapping_dict[name_component] = dict()
                    current_variable_dict = current_variable_dict[name_component]
                    current_variable_mapping_dict = current_variable_mapping_dict[name_component]
            name_component = full_variable_name[slash_indices[-1]:]
            current_variable_dict[name_component] = doc[full_variable_name]
            current_variable_mapping_dict[name_component] = full_variable_name
        return variable_dict, variable_mapping_dict

    def __generate_class_definitions(self, db_name, collection_name,
                                     variables, variable_mappings,
                                     class_name, class_definitions):
        if type(variables).__name__ != 'dict':
            return

        class_definitions[class_name]['db_name'] = db_name
        class_definitions[class_name]['collection_name'] = collection_name
        class_definitions[class_name]['variables'] = list()
        class_definitions[class_name]['objects'] = list()
        class_definitions[class_name]['variable_mappings'] = dict()
        for variable in variables:
            if type(variables[variable]).__name__ != 'dict':
                if variable != '_id':
                    class_definitions[class_name][variable] = variables[variable]
                else:
                    class_definitions[class_name][variable] = str(variables[variable])
                class_definitions[class_name]['variables'].append(variable)
                class_definitions[class_name]['variable_mappings'][variable] = variable_mappings[variable]
            else:
                new_class_name = variable.title()
                class_definitions[class_name][variable] = new_class_name + '()'
                class_definitions[class_name]['objects'].append(variable)
                class_definitions[new_class_name] = dict()
                self.__generate_class_definitions(db_name,
                                                  collection_name,
                                                  variables[variable],
                                                  variable_mappings[variable],
                                                  new_class_name,
                                                  class_definitions)

    def __generate_source_file(self, db_name, collection_name, class_definitions):
        script_source = '# This file has been autogenerated by black_box_odm\n'
        script_source += 'import pymongo as pm\n\n'
        for class_name in class_definitions:
            class_str = 'class {0}(object):\n'.format(class_name)
            class_str += '    db_name = "{}"\n'.format(db_name)
            class_str += '    collection_name = "{}"\n\n'.format(collection_name)

            class_str += '    def __init__(self):\n'
            for var in class_definitions[class_name]:
                if var == 'db_name' or var == 'collection_name':
                    continue

                val = class_definitions[class_name][var]
                if type(val).__name__ == 'str' and '()' not in val:
                    class_str += '        self.{0} = "{1}"\n'.format(var, val)
                else:
                    class_str += '        self.{0} = {1}\n'.format(var, val)

            class_str += '        self.mongo_client = pm.MongoClient()\n'
            class_str += '\n'

            class_str += '    @staticmethod\n'
            class_str += '    def get_all():\n'
            class_str += '        client = pm.MongoClient()\n'
            class_str += '        db = client[{0}.db_name]\n'.format(class_name)
            class_str += '        collection = db[{0}.collection_name]\n'.format(class_name)
            class_str += '        cursor = collection.find()\n'
            class_str += '        data = list()\n'
            class_str += '        for doc in cursor:\n'
            class_str += '            obj = {0}()\n'.format(class_name)
            for var in class_definitions[class_name]:
                if var in class_definitions[class_name]['variables']:
                    var_mapping = class_definitions[class_name]['variable_mappings'][var]
                    class_str += '            obj.{0} = doc["{1}"]\n'.format(var, var_mapping)
                elif var in class_definitions[class_name]['objects']:
                    class_str += '            obj.{0}.from_doc(doc)\n'.format(var)
            class_str += '            data.append(obj)\n'
            class_str += '        return data\n\n'

            class_str += '    def from_doc(self, doc):\n'
            for var in class_definitions[class_name]:
                if var in class_definitions[class_name]['variables']:
                    var_mapping = class_definitions[class_name]['variable_mappings'][var]
                    class_str += '        self.{0} = doc["{1}"]\n'.format(var, var_mapping)
                elif var in class_definitions[class_name]['objects']:
                    class_str += '        self.{0}.from_doc(doc)\n'.format(var)
            script_source += '{0}\n\n'.format(class_str)

        with open(collection_name + '.py', 'w') as source_file:
            source_file.write(script_source)
