import time
from importlib import import_module
import rospy
from rospy_message_converter.message_converter import convert_ros_message_to_dictionary

class ROSTopicListener(object):
    '''A generic ROS topic listener.

    Constructor arguments:
    @param topic_name -- name of a topic to which we want to subscribe
    @param msg_pkg -- name of the package in which the message type of the topic is defined
    @param msg_type -- name of the message type
    @param max_frequency -- maximum frequency at which the subscriber should listen to messages
    @param data_logger -- a black_box.loggers.LoggerBase instance
    @param interface_name -- name of the data source; prepended to the topic name for defining
                             the full name of the logged data collection (default "ros")

    @author Alex Mitrevski, Santosh Thoduka
    @contact aleksandar.mitrevski@h-brs.de, santosh.thoduka@h-brs.de

    '''
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
        '''Creates a topic subscriber for "self.topic_name" of type "self.msg_type".
        '''
        msg_module = import_module(self.msg_pkg)
        msg_class = getattr(msg_module, self.msg_type)
        self.subscriber = rospy.Subscriber(self.topic_name, msg_class, self.log_msg)

    def shutdown(self):
        '''Shuts down the topic subscriber.
        '''
        if self.subscriber:
            self.subscriber.unregister()

    def log_msg(self, msg):
        '''Passes the received message on to the data logger (in a dictionary form).
        '''
        dict_msg = convert_ros_message_to_dictionary(msg)
        timestamp = time.time()
        self.data_logger.log_data(self.variable_name, timestamp, dict_msg)
