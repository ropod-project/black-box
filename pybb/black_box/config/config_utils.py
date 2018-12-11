class ConfigUtils(object):
    @staticmethod
    def get_full_variable_name(data_source_name, var_name):
        '''Creates a variable name for "var_name" that includes the data source name.
        The resulting name is "data_source_name"_"var_name" where the underscore
        is used as a separator, but every instance of / in "var_name" is replaced
        by an underscore. A forward slash in "var_name" is removed from it.

        Examples:
        If "data_source_name" is "ros" and "var_name" is "/laser/scan",
        the returned variable name will be value "ros_laser_scan".

        Keyword arguments:
        @param data_source_name -- name of a data source (e.g. ros, zyre)
        @param var_name -- name of a variable to be logged

        '''
        if var_name[0] == '/':
            var_name = var_name[1:]
        var_name = var_name.replace('/', '_')
        full_var_name = '{0}_{1}'.format(data_source_name, var_name)
        return full_var_name
