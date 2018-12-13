import time
import threading
import json
import zmq
from black_box.config.config_utils import ConfigUtils

class JsonZmqReader(object):
    '''A listener for JSON messages published via ZMQ.

    Constructor arguments:
    @param url -- publisher URL (starting with tcp://)
    @param port -- subscriber port
    @param topic_params -- a list of black_box.config.config_params.ZmqTopicParams instances
    @param data_logger -- a black_box.loggers.LoggerBase instance
    @param interface_name -- name of the data source; prepended to the topic name for defining
                             the full name of the logged data collection (default "zmq")

    @author Alex Mitrevski, Santosh Thoduka
    @contact aleksandar.mitrevski@h-brs.de, santosh.thoduka@h-brs.de

    '''
    def __init__(self, url, port, topic_params, data_logger, interface_name='zmq'):
        self.data_logger = data_logger
        self.publisher_url = url
        self.port = port
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.topic_names = [topic.name for topic in topic_params]
        self.variable_names = {topic:ConfigUtils.get_full_variable_name(interface_name, topic)
                               for topic in self.topic_names}
        self.min_times_between_msgs = {topic.name:(1./topic.max_frequency)
                                       for topic in topic_params}
        self.previous_msg_times = {topic:-1 for topic in self.topic_names}

        for topic in self.topic_names:
            self.socket.setsockopt_string(zmq.SUBSCRIBE, topic)
        self.sub_thread = None
        self.subscriber_running = False

    def start(self):
        '''Starts a ZMQ subscriber socket and runs the subscriber on a background thread.
        '''
        if not self.sub_thread:
            self.socket.connect('{0}:{1}'.format(self.publisher_url, self.port))
            self.sub_thread = threading.Thread(target=self.log_msg)
            for topic in self.topic_names:
                self.previous_msg_times[topic] = time.time()
            self.subscriber_running = True
            self.sub_thread.start()

    def stop(self):
        '''Stops the subscriber thread.
        '''
        if self.sub_thread:
            self.subscriber_running = False
            self.sub_thread.join()

    def log_msg(self):
        '''Logs incoming messages as long as self.subscriber_running is True.
        '''
        while self.subscriber_running:
            topic, msg = self.socket.recv_multipart()
            try:
                topic = topic.decode('utf-8')
                msg = json.loads(msg.decode('utf-8'))
                if self.min_time_elapsed(topic):
                    timestamp = time.time()
                    self.data_logger.log_data(self.variable_names[topic], timestamp, msg)
                    self.previous_msg_times[topic] = time.time()
            except json.JSONDecodeError:
                print('[json_zmq_reader] Invalid JSON message received')

    def min_time_elapsed(self, topic):
        '''Returns True if the elapsed time since the last message on the given topic is
        greater than the minimum allowed time between messages; returns False otherwise.

        Keyword arguments:
        @param topic -- name of a message topic

        '''
        elapsed_time = time.time() - self.previous_msg_times[topic]
        return elapsed_time > self.min_times_between_msgs[topic]
