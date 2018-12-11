class ConfigKeys(object):
    DEFAULT_PARAMETERS = 'default_parameters'
    ROS = 'ros'
    ZYRE = 'zyre'

class DataSourceNames(object):
    ROS = 'ros'
    ZYRE = 'zyre'

class DefaultParams(object):
    def __init__(self):
        self.max_frequency = 0
        self.max_db_size = 0
        self.split_db = False

class RosTopicParams(object):
    def __init__(self):
        self.name = ''
        self.msg_pkg = ''
        self.msg_type = ''
        self.max_frequency = 0
        self.metadata = None

class RosParams(object):
    def __init__(self):
        self.ros_master_uri = 'http://localhost:11311'
        self.topic = []

class ZyreParams(object):
    def __init__(self):
        self.node_name = ''
        self.groups = []
        self.message_types = []

class RosMetadataParams(object):
    def __init__(self):
        self.topic_name = ''
        self.msg_type = ''
        self.direct_msg_mapping = True

class ConfigParams(object):
    def __init__(self):
        self.default = DefaultParams()
        self.ros = RosParams()
        self.zyre = ZyreParams()

    def __str__(self):
        obj_str = ''
        obj_str += 'default:\n'
        obj_str += '    max_frequency: {0}\n'.format(self.default.max_frequency)
        obj_str += '    max_db_size: {0}\n'.format(self.default.max_db_size)
        obj_str += '    split_db: {0}\n'.format(self.default.split_db)

        obj_str += 'ros_params:\n'
        obj_str += '    ros_master_uri: {0}\n'.format(self.ros.ros_master_uri)
        obj_str += '    topic_params:\n'
        for i, topic_params in enumerate(self.ros.topic):
            obj_str += '        {0}:\n'.format(i)
            obj_str += '            name: {0}\n'.format(topic_params.name)
            obj_str += '            msg_pkg: {0}\n'.format(topic_params.msg_pkg)
            obj_str += '            msg_type: {0}\n'.format(topic_params.msg_type)
            obj_str += '            max_frequency: {0}\n'.format(topic_params.max_frequency)
            obj_str += '            metadata:\n'
            obj_str += '                ros:\n'
            obj_str += '                    topic_name: {0}\n'.format(topic_params.metadata.topic_name)
            obj_str += '                    msg_type: {0}\n'.format(topic_params.metadata.msg_type)
            obj_str += '                    direct_msg_mapping: {0}\n'.format(topic_params.metadata.direct_msg_mapping)

        obj_str += 'zyre_params:\n'
        obj_str += '    node_name: {0}\n'.format(self.zyre.node_name)
        obj_str += '    groups: {0}\n'.format(self.zyre.groups)
        obj_str += '    message_types: {0}'.format(self.zyre.message_types)
        return obj_str
