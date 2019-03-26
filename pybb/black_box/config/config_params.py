class ConfigKeys(object):
    DEFAULT_PARAMETERS = 'default_parameters'
    ROS = 'ros'
    ZYRE = 'zyre'
    ZMQ = 'zmq'

class DataSourceNames(object):
    ROS = 'ros'
    ZYRE = 'zyre'
    ZMQ = 'zmq'

class DefaultParams(object):
    def __init__(self):
        self.max_frequency = 0
        self.max_db_size = 0
        self.split_db = False
        self.db_name = 'logs'

class RosTopicParams(object):
    def __init__(self):
        self.name = ''
        self.msg_pkg = ''
        self.msg_type = ''
        self.max_frequency = 0
        self.metadata = None
    def to_dict(self) :
        return {'name': self.name, 
                'msg_pkg' : self.msg_pkg,
                'msg_type' : self.msg_type,
                'max_frequency' : self.max_frequency,
                'metadata' : self.metadata }
    def from_dict(self, param_dict) :
        self.name = param_dict['name']
        self.msg_pkg = param_dict['msg_pkg']
        self.msg_type = param_dict['msg_type']
        self.max_frequency = param_dict['max_frequency']
        self.metadata = param_dict['metadata']

class RosParams(object):
    def __init__(self):
        self.ros_master_uri = 'http://localhost:11311'
        self.topic = []

class ZyreParams(object):
    def __init__(self):
        self.node_name = ''
        self.groups = []
        self.message_types = []

class ZmqTopicParams(object):
    def __init__(self):
        self.name = ''
        self.max_frequency = 0
        self.metadata = None

class ZmqParams(object):
    def __init__(self):
        self.url = ''
        self.port = -1
        self.topics = []

class RosMetadataParams(object):
    def __init__(self):
        self.topic_name = ''
        self.msg_type = ''
        self.direct_msg_mapping = True

class ConfigParams(object):
    def __init__(self):
        self.default = DefaultParams()
        self.ros = None
        self.zyre = None
        self.zmq = None

    def __str__(self):
        obj_str = ''
        obj_str += 'default:\n'
        obj_str += '    max_frequency: {0}\n'.format(self.default.max_frequency)
        obj_str += '    max_db_size: {0}\n'.format(self.default.max_db_size)
        obj_str += '    split_db: {0}\n'.format(self.default.split_db)

        obj_str += 'ros:\n'
        obj_str += '    ros_master_uri: {0}\n'.format(self.ros.ros_master_uri)
        obj_str += '    topic_params:\n'
        for i, topic_params in enumerate(self.ros.topic):
            obj_str += '        {0}:\n'.format(i)
            obj_str += '            name: {0}\n'.format(topic_params.name)
            obj_str += '            msg_pkg: {0}\n'.format(topic_params.msg_pkg)
            obj_str += '            msg_type: {0}\n'.format(topic_params.msg_type)
            obj_str += '            max_frequency: {0}\n'.format(topic_params.max_frequency)
            if topic_params.metadata:
                obj_str += '            metadata:\n'
                obj_str += '                ros:\n'
                obj_str += '                    topic_name: {0}\n'.format(topic_params.metadata.topic_name)
                obj_str += '                    msg_type: {0}\n'.format(topic_params.metadata.msg_type)
                obj_str += '                    direct_msg_mapping: {0}\n'.format(topic_params.metadata.direct_msg_mapping)

        obj_str += 'zyre:\n'
        obj_str += '    node_name: {0}\n'.format(self.zyre.node_name)
        obj_str += '    groups: {0}\n'.format(self.zyre.groups)
        obj_str += '    message_types: {0}\n'.format(self.zyre.message_types)

        obj_str += 'zmq:\n'
        obj_str += '    url: {0}\n'.format(self.zmq.url)
        obj_str += '    port: {0}\n'.format(self.zmq.port)
        obj_str += '    topic_params:\n'
        for i, topic_params in enumerate(self.zmq.topics):
            obj_str += '        {0}:\n'.format(i)
            obj_str += '            name: {0}\n'.format(topic_params.name)
            obj_str += '            max_frequency: {0}\n'.format(topic_params.max_frequency)
            if topic_params.metadata:
                obj_str += '            metadata:\n'
                obj_str += '                ros:\n'
                obj_str += '                    topic_name: {0}\n'.format(topic_params.metadata.topic_name)
                obj_str += '                    msg_type: {0}\n'.format(topic_params.metadata.msg_type)
                obj_str += '                    direct_msg_mapping: {0}\n'.format(topic_params.metadata.direct_msg_mapping)
        return obj_str
