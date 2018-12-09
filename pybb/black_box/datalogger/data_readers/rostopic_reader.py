import rospy
from black_box.datalogger.data_readers.reader_base import DataReaderBase
from black_box.datalogger.rostopic_listener import ROSTopicListener

class ROSTopicReader(DataReaderBase):
    def __init__(self, node_handle_name, config_params, max_frequency, data_logger):
        super(ROSTopicReader, self).__init__(data_logger)
        self.node_handle_name = node_handle_name
        self.config_params = config_params
        self.max_frequency = max_frequency
        self.listeners = []

        self.start_node()
        self.init_listeners(data_logger)

    def start(self):
        sleep_time_s = 1. / self.max_frequency
        self.start_listeners()
        while not rospy.is_shutdown():
            rospy.sleep(sleep_time_s)

    def stop(self):
        for listener in self.listeners:
            listener.shutdown()

    def start_node(self):
        rospy.init_node(self.node_handle_name)

    def init_listeners(self, topic):
        self.listeners = []
        for topic_params in self.config_params.topic:
            listener = ROSTopicListener(topic_params.name,
                                        topic_params.msg_pkg,
                                        topic_params.msg_type,
                                        topic_params.max_frequency,
                                        self.data_logger)
            self.listeners.append(listener)

    def start_listeners(self):
        for listener in self.listeners:
            listener.start()
