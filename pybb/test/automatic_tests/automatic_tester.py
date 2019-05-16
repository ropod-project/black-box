#! /usr/bin/env python3
"""This module is a test which reads config file and creates publishers accordingly.
After all publisher finishes publishing, it checks if blackbox was able to log
all the messages or not and prints status."""

from __future__ import print_function
import threading
import sys
import psutil
import rospy
from termcolor import colored

from rostopic_publisher import RosTopicPublisher
from zyre_publisher import ZyrePublisher

from black_box.config.config_file_reader import ConfigFileReader
from black_box.config.config_utils import ConfigUtils
from black_box_tools.db_utils import DBUtils

class AutomaticTester():
    '''An interface to manage all publishers

    Constructor arguments:
    @param config_params -- an instance of black_box.config.config_params

    '''
    def __init__(self, config_params, duration):
        self.config_params = config_params
        self.duration = duration
        self.publishers = []
        self.publisher_threads = []

        if self.config_params.ros:
            rospy.init_node('automatic_tester')

            # initialising ros publishers
            for topic_params in self.config_params.ros.topic:
                num_of_msgs = self.duration * topic_params.max_frequency
                publisher = RosTopicPublisher(
                    topic_params.name,
                    topic_params.msg_pkg,
                    topic_params.msg_type,
                    num_of_msgs=num_of_msgs,
                    max_frequency=topic_params.max_frequency,
                    )
                pub_thread = threading.Thread(target=publisher.start_publishing)
                self.publishers.append(publisher)
                self.publisher_threads.append(pub_thread)

        if self.config_params.zyre:
            for message_type in self.config_params.zyre.message_types:
                num_of_msgs = self.duration * self.config_params.default.max_frequency
                publisher = ZyrePublisher(
                    message_type,
                    self.config_params.zyre.groups,
                    num_of_msgs=num_of_msgs,
                    max_frequency=self.config_params.default.max_frequency
                    )
                pub_thread = threading.Thread(target=publisher.start_publishing)
                self.publishers.append(publisher)
                self.publisher_threads.append(pub_thread)

    def start(self):
        '''Starts and runs the publishers on background threads
        '''
        for pub_thread in self.publisher_threads:
            pub_thread.start()

    def stop(self):
        """Wait for all pub to complete.
        :returns: None

        """
        for pub_thread in self.publisher_threads:
            pub_thread.join()

def is_bb_running():
    """Checks in all processes if black box is one of the process or not
    :returns: bool

    """
    processes = list(psutil.process_iter())
    bb_processes = [process for process in processes if 'logger_main.py' in process.cmdline()]
    return len(bb_processes) > 0

def check_logs(config_params, test_duration):
    """Check the logs in mongodb and print the status.

    :config_params: black_box.config.ConfigParams
    :test_duration: float
    :returns: None

    """
    db_name = config_params.default.db_name
    collection_names = DBUtils.get_data_collection_names(db_name)

    # check if all topics are present in db
    fail = False
    size_status = []
    if config_params.ros:
        for topic_params in config_params.ros.topic:
            topic_name = ConfigUtils.get_full_variable_name("ros", topic_params.name)
            if topic_name not in collection_names:
                fail = True
                print(colored(topic_name + " not present in mongoDB", "red"))
                size_status.append({
                    'collection': topic_name,
                    'expected_size':topic_params.max_frequency*test_duration,
                    'collection_size':0})
                continue
            collection_size = len(DBUtils.get_all_docs(db_name, topic_name))
            size_status.append({
                'collection': topic_name,
                'expected_size':topic_params.max_frequency*test_duration,
                'collection_size':collection_size})
    if config_params.zyre:
        expected_size = test_duration * config_params.default.max_frequency
        for message_type in config_params.zyre.message_types:
            topic_name = ConfigUtils.get_full_variable_name("zyre", message_type)
            if topic_name not in collection_names:
                fail = True
                print(colored(topic_name + " not present in mongoDB", "red"))
                size_status.append({
                    'collection': topic_name,
                    'expected_size': expected_size,
                    'collection_size': 0})
                continue
            collection_size = len(DBUtils.get_all_docs(db_name, topic_name))
            size_status.append({
                'collection': topic_name,
                'expected_size': expected_size,
                'collection_size': collection_size})
    if not fail:
        print(colored("All topics have their respective collection in mongoDB", "green"))
    for comparison in size_status:
        color = "green" if comparison['expected_size'] == comparison['collection_size'] else "red"
        string = comparison['collection'] + ': ' + str(comparison['collection_size']) \
                + '/' + str(comparison['expected_size'])
        print(colored(string, color))

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print('Usage: python3 automatic_tester.py [absolute-path-to-black-box-config-file]')
        sys.exit(1)
    BB_CONFIG_FILE = sys.argv[1]

    TEST_DURATION = 20 #seconds

    # only proceed if black box is running
    if not is_bb_running():
        print('Blackbox is not running. Please make sure it is running before',
              'executing this test script.')
        sys.exit(1)

    CONFIG_PARAMS = ConfigFileReader.load_config(BB_CONFIG_FILE)
    DBUtils.clear_db(CONFIG_PARAMS.default.db_name)

    TESTER = AutomaticTester(CONFIG_PARAMS, TEST_DURATION)
    print("initialised all publisher")

    TESTER.start()
    print("publishers running")

    TESTER.stop()
    print("publishers stopped")

    check_logs(CONFIG_PARAMS, TEST_DURATION)
