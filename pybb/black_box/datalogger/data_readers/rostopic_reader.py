from multiprocessing import Process, Queue
import rospy
import rosnode
from black_box.datalogger.data_readers.rostopic_listener import ROSTopicListener
from black_box.config.config_params import RosTopicParams
from black_box.config.config_utils import ConfigUtils

class ROSTopicReader(object):
    '''An interface for managing ROS topic listeners.

    Constructor arguments:
    @param node_handle_name -- name of the ROS listener node
    @param config_params -- an instance of black_box.config.config_params.RosParams
    @param max_frequency -- maximum frequency at which the node should be running
    @param data_logger -- a black_box.loggers.LoggerBase instance

    @author Alex Mitrevski, Santosh Thoduka, Dharmin B.
    @contact aleksandar.mitrevski@h-brs.de, santosh.thoduka@h-brs.de

    '''
    def __init__(self, node_handle_name, config_params, max_frequency, data_logger):
        self.node_handle_name = node_handle_name
        self.config_params = config_params
        self.max_frequency = max_frequency
        self.data_logger = data_logger
        self.listeners = []
        self.listeners_initialised = False
        self.node_thread = []
        self.queue = Queue()

    def start(self):
        '''Sleeps for (1. / self.max_frequency) until interrupted. Implements
        a recovery behaviour for dealing with a ROS master that has not been
        started yet or that dies during the execution, namely waits until
        there is no master and (re)initialises the listener when the master comes up.
        If the master is working and listeners are not initialised then a process
        for each rostopic is started.
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
                    for topic_params in self.config_params.topic:
                        self.queue.put(topic_params.to_dict())
                        process = Process(target=self.__create_node, args=(self.queue,))
                        process.start()
                        self.node_thread.append(process)
                    self.listeners_initialised = True
            rospy.sleep(sleep_time_s)

    def stop(self):
        '''Stops all topic listeners and kills the listener node.
        '''
        self.__terminate_node()

    def __create_node(self, queue):
        '''Starts a ROS node with the name "self.node_handle_name",
        initialises listeners for the topics in "self.config_params.topic",
        and blocks the execution (expected to be run as a background process).

        @param queue -- a queue object to get the parameters from parent process
        '''
        topic_params_dict = queue.get()
        topic_params = RosTopicParams()
        topic_params.from_dict(topic_params_dict)
        self.new_handle_name = ConfigUtils.get_full_variable_name(
                "ros_logger", topic_params.name)
        rospy.init_node(self.new_handle_name)
        print('[rostopic_reader] {0} initialised'.format(self.new_handle_name))
        self.listener = ROSTopicListener(topic_params.name,
                                        topic_params.msg_pkg,
                                        topic_params.msg_type,
                                        topic_params.max_frequency,
                                        self.data_logger)
        self.listener.start()
        try:
            rospy.spin()
        except TypeError:
            self.listener.shutdown()
            print('[rostopic_reader] {0} terminated'.format(self.new_handle_name))

    def __terminate_node(self):
        if self.listeners_initialised:
            for topic_params in self.config_params.topic:
                rosnode.kill_nodes(self.node_handle_name + topic_params.name[1:])
            for process in self.node_thread :
                print(process.pid)
                process.terminate()

    def __is_master_running(self):
        '''Returns True if a ROS master is running; returns False otherwise.
        '''
        try:
            rospy.get_master().getPid()
            return True
        except:
            return False
