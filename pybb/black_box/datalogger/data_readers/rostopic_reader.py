from multiprocessing import Process
import rospy
import rosnode
from black_box.datalogger.data_readers.reader_base import DataReaderBase
from black_box.datalogger.rostopic_listener import ROSTopicListener

class ROSTopicReader(DataReaderBase):
    '''An interface for managing ROS topic listeners.

    Constructor arguments:
    @param node_handle_name -- name of the ROS listener node
    @param config_params -- an instance of black_box.config.config_params.RosParams
    @param max_frequency -- maximum frequency at which the node should be running
    @data_logger -- a black_box.loggers.LoggerBase instance

    @author Alex Mitrevski, Santosh Thoduka
    @contact aleksandar.mitrevski@h-brs.de, santosh.thoduka@h-brs.de

    '''
    def __init__(self, node_handle_name, config_params, max_frequency, data_logger):
        super(ROSTopicReader, self).__init__(data_logger)
        self.node_handle_name = node_handle_name
        self.config_params = config_params
        self.max_frequency = max_frequency
        self.listeners = []
        self.listeners_initialised = False
        self.node_thread = None

    def start(self):
        '''Sleeps for (1. / self.max_frequency) until interrupted. Implements
        a recovery behaviour for dealing with a ROS master that has not been
        started yet or that dies during the execution, namely waits until
        there is no master and (re)initialises the listener when the master comes up.
        '''
        sleep_time_s = 1. / self.max_frequency
        while not rospy.is_shutdown():
            if not self.__is_master_running():
                print('[rostopic_reader] The ROS master appears to have died or has not been started...')
                self.__terminate_node()
                self.listeners_initialised = False
                while not self.__is_master_running():
                    rospy.sleep(sleep_time_s)
            else:
                if not self.listeners_initialised:
                    print('[rostopic_reader] Connection with ROS master established')
                    self.node_thread = Process(target=self.__create_node)
                    self.node_thread.start()
                    self.listeners_initialised = True
            rospy.sleep(sleep_time_s)

    def stop(self):
        '''Stops all topic listeners and kills the listener node.
        '''
        self.__stop_listeners()
        self.__terminate_node()

    def __create_node(self):
        '''Starts a ROS node with the name "self.node_handle_name",
        initialises listeners for the topics in "self.config_params.topic",
        and blocks the execution (expected to be run as a background process).
        '''
        rospy.init_node(self.node_handle_name)
        self.__init_listeners()
        rospy.spin()

    def __terminate_node(self):
        if self.listeners_initialised:
            rosnode.kill_nodes(self.node_handle_name)
            self.node_thread.terminate()

    def __init_listeners(self):
        '''Initialises listeners for the topics in "self.config_params.topic";
        "self.listeners" is populated with the listeners after this call.
        '''
        self.listeners = []
        for topic_params in self.config_params.topic:
            listener = ROSTopicListener(topic_params.name,
                                        topic_params.msg_pkg,
                                        topic_params.msg_type,
                                        topic_params.max_frequency,
                                        self.data_logger)
            listener.start()
            self.listeners.append(listener)

    def __stop_listeners(self):
        '''Shuts down all listeners in "self.listeners".
        '''
        if self.listeners_initialised:
            for listener in self.listeners:
                listener.shutdown()

    def __is_master_running(self):
        '''Returns True if a ROS master is running; returns False otherwise.
        '''
        try:
            rospy.get_master().getPid()
            return True
        except:
            return False
