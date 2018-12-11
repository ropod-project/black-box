import time
from importlib import import_module
from rospy_message_converter.message_converter import convert_ros_message_to_dictionary
import rospy
from black_box.config.config_utils import ConfigUtils

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
        self.previous_msg_time = time.time()
        self.min_time_between_msgs = 1. / max_frequency
        self.variable_name = ConfigUtils.get_full_variable_name(interface_name, self.topic_name)

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

        @param msg -- a ROS message of type "self.msg_type"

        '''
        if self.min_time_elapsed():
            dict_msg = convert_ros_message_to_dictionary(msg)
            timestamp = time.time()
            self.data_logger.log_data(self.variable_name, timestamp, dict_msg)
            self.previous_msg_time = timestamp

    def min_time_elapsed(self):
        '''Returns True if the elapsed time since the last message is
        greater than the minimum allowed time between messages; returns False otherwise.
        '''
        elapsed_time = time.time() - self.previous_msg_time
        return elapsed_time > self.min_time_between_msgs
