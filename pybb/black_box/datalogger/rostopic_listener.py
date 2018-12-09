import time
from importlib import import_module
import rospy
from rospy_message_converter.message_converter import convert_ros_message_to_dictionary

class ROSTopicListener(object):
    def __init__(self, topic_name, msg_pkg, msg_type,
                 max_frequency, data_logger, interface_name='ros'):
        self.topic_name = topic_name
        self.msg_pkg = msg_pkg
        self.msg_type = msg_type
        self.max_frequency = max_frequency
        self.data_logger = data_logger
        self.subscriber = None

        self.variable_name = self.topic_name
        if self.variable_name[0] == '/':
            self.variable_name = self.variable_name[1:]
        self.variable_name = self.variable_name.replace('/', '_')
        self.variable_name = '{0}_{1}'.format(interface_name, self.variable_name)

    def start(self):
        msg_module = import_module(self.msg_pkg)
        msg_class = getattr(msg_module, self.msg_type)
        self.subscriber = rospy.Subscriber(self.topic_name, msg_class, self.log_msg)

    def shutdown(self):
        if self.subscriber:
            self.subscriber.unregister()

    def log_msg(self, msg):
        dict_msg = convert_ros_message_to_dictionary(msg)
        timestamp = time.time()
        self.data_logger.log_data(self.variable_name, timestamp, dict_msg)
