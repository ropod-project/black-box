# ==============================================
class ConfigKeys(object):
    DEFAULT_PARAMETERS = 'default_parameters'
    ROS = 'ros'
    ZYRE = 'zyre'
    ZMQ = 'zmq'
    EVENT = 'event'

    def __str__(self):
        obj_str = ''
        obj_str += self.__class__.__name__ + '\n'
        return obj_str

# ==============================================
class DataSourceNames(object):
    ROS = 'ros'
    ZYRE = 'zyre'
    ZMQ = 'zmq'
    EVENT = 'event'

    def __str__(self):
        obj_str = ''
        obj_str += self.__class__.__name__ + '\n'
        return obj_str

# ==============================================
class DefaultParams(object):
    def __init__(self):
        self.max_frequency = 0
        self.max_db_size = 0
        self.split_db = False
        self.db_name = 'logs'

    def __str__(self):
        obj_str = ''
        obj_str += self.__class__.__name__ + '\n'
        return obj_str
    def __str__(self):
        obj_str = ''
        obj_str += 'max_frequency: {0}\n'.format(self.max_frequency)
        obj_str += 'max_db_size: {0}\n'.format(self.max_db_size)
        obj_str += 'split_db: {0}\n'.format(self.split_db)
        obj_str += 'db_name: {0}\n'.format(self.db_name)
        return obj_str

# ==============================================
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

    def __str__(self):
        obj_str = ''
        # obj_str += self.__class__.__name__ + '\n'
        obj_str += 'name: {0}\n'.format(self.name)
        obj_str += 'msg_pkg: {0}\n'.format(self.msg_pkg)
        obj_str += 'msg_type: {0}\n'.format(self.msg_type)
        obj_str += 'max_frequency: {0}\n'.format(self.max_frequency)
        if self.metadata:
            obj_str += 'metadata:\n'
            obj_str += '\t'
            obj_str += str(self.metadata).replace('\n', '\n\t')[:-1]
        return obj_str

# ==============================================
class RosParams(object):
    def __init__(self):
        self.ros_master_uri = 'http://localhost:11311'
        self.topic = []

    def __str__(self):
        obj_str = ''
        # obj_str += self.__class__.__name__ + '\n'
        obj_str += 'ros_master_uri: {0}\n'.format(self.ros_master_uri)
        obj_str += 'topic_params:\n'
        for i, topic_params in enumerate(self.topic):
            obj_str += '\t{0}:\n'.format(i)
            obj_str += '\t'
            obj_str += str(topic_params).replace('\n', '\n\t')[:-1]
        return obj_str

# ==============================================
class RosMetadataParams(object):
    def __init__(self):
        self.topic_name = ''
        self.msg_type = ''
        self.direct_msg_mapping = True

    def __str__(self):
        obj_str = ''
        # obj_str += self.__class__.__name__ + '\n'
        obj_str += 'ros:\n'
        obj_str += '\ttopic_name: {0}\n'.format(self.topic_name)
        obj_str += '\tmsg_type: {0}\n'.format(self.msg_type)
        obj_str += '\tdirect_msg_mapping: {0}\n'.format(self.direct_msg_mapping)
        return obj_str

# ==============================================
class ZyreParams(object):
    def __init__(self):
        self.node_name = ''
        self.groups = []
        self.message_types = []

    def __str__(self):
        obj_str = ''
        # obj_str += self.__class__.__name__ + '\n'
        obj_str += 'node_name: {0}\n'.format(self.node_name)
        obj_str += 'groups: {0}\n'.format(self.groups)
        obj_str += 'message_types: {0}\n'.format(self.message_types)
        return obj_str

# ==============================================
class EventParams(object):
    def __init__(self):
        self.listeners = []

    def __str__(self):
        obj_str = ''
        # obj_str += self.__class__.__name__ + '\n'
        obj_str += 'listerners:\n'
        for i, listener in enumerate(self.listeners):
            obj_str += '\t{0}:\n'.format(i)
            obj_str += '\t'
            obj_str += str(listener).replace('\n', '\n\t')[:-1]
        return obj_str

# ==============================================
class EventListenerParams(object):
    def __init__(self):
        self.name = ''
        self.event_type = 'CHANGE'
        self.max_frequency = 0

    def __str__(self):
        obj_str = ''
        # obj_str += self.__class__.__name__ + '\n'
        obj_str += 'name: {0}\n'.format(self.name)
        obj_str += 'event_type: {0}\n'.format(self.event_type)
        obj_str += 'max_frequency: {0}\n'.format(self.max_frequency)
        return obj_str

# ==============================================
class ZmqTopicParams(object):
    def __init__(self):
        self.name = ''
        self.max_frequency = 0
        self.metadata = None

    def __str__(self):
        obj_str = ''
        obj_str += self.__class__.__name__ + '\n'
        obj_str += 'name: {0}\n'.format(self.name)
        obj_str += 'max_frequency: {0}\n'.format(self.max_frequency)
        if self.metadata:
            obj_str += 'metadata:\n'
            obj_str += '\t'
            obj_str += str(self.metadata).replace('\n', '\n\t')[:-1]
        return obj_str

# ==============================================
class ZmqParams(object):
    def __init__(self):
        self.url = ''
        self.port = -1
        self.topics = []

    def __str__(self):
        obj_str = ''
        obj_str += self.__class__.__name__ + '\n'
        obj_str += 'url: {0}\n'.format(self.url)
        obj_str += 'port: {0}\n'.format(self.port)
        obj_str += 'topic_params:\n'
        for i, topic_params in enumerate(self.topics):
            obj_str += '\t{0}:\n'.format(i)
            obj_str += '\t'
            obj_str += str(topic_params).replace('\n', '\n\t')[:-1]
        return obj_str

# ==============================================
class ConfigParams(object):
    def __init__(self):
        self.default = DefaultParams()
        self.ros = None
        self.zyre = None
        self.zmq = None
        self.event = None

    def __str__(self):
        obj_str = ''
        obj_str += self.__class__.__name__ + '\n'
        obj_str += 'default:\n\t'
        obj_str += str(self.default).replace('\n', '\n\t')[:-1]

        if self.ros:
            obj_str += '\nros:\n\t'
            obj_str += str(self.ros).replace('\n', '\n\t')[:-1]

        if self.zyre:
            obj_str += '\nzyre:\n\t'
            obj_str += str(self.zyre).replace('\n', '\n\t')[:-1]

        if self.zmq:
            obj_str += '\nzmq:\n\t'
            obj_str += str(self.zmq).replace('\n', '\n\t')[:-1]

        if self.event:
            obj_str += '\nevent:\n\t'
            obj_str += str(self.event).replace('\n', '\n\t')[:-1]
        return obj_str

