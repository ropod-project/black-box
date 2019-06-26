#! /usr/bin/env python3
"""This module contains unit tests to evaluate the logging capabilities of black
box using automatic test"""

import time
import os
import signal
import subprocess
import unittest

from black_box.automatic_tests.automatic_tester import main as automatic_tester_main

class TestBBLogging(unittest.TestCase):

    """ Unit Tests for black box using automated tests"""

    @classmethod
    def setUpClass(cls):
        cls.threshold = 0.9 # percent of message that are required to be logged

        test_dir = os.path.abspath(os.path.dirname(__file__))
        pybb_dir = os.path.dirname(test_dir)
        main_dir = os.path.dirname(pybb_dir)
        bb_program_file = os.path.join(pybb_dir, 'logger_main.py')
        cls.config_file_path = os.path.join(main_dir, 'config/test_sources.yaml')

        bb_output_file_path = '/tmp/bb_output_file_' + str(time.time()).replace('.', '_')
        cls.bb_output_file = open(bb_output_file_path, 'w')

        print("Starting black box as a process")
        commands = [str(bb_program_file), str(cls.config_file_path)]
        cls.process = subprocess.Popen(commands, stdout=cls.bb_output_file)
        print("Waiting for black box process to initialise completely...")
        time.sleep(10)

    @classmethod
    def tearDownClass(cls):
        cls.process.send_signal(signal.SIGINT)
        cls.process.wait()
        cls.bb_output_file.close()

    def test_bb_with_test_config(self):
        """Test black box with test_sources.yaml config file"""

        print("Starting test...")
        output = automatic_tester_main(self.config_file_path, 10)
        for topic in output:
            string = topic['collection'] + ': ' + str(topic['collection_size']) \
                + '/' + str(topic['expected_size'])
            print(string)
            self.assertGreaterEqual(
                float(topic['collection_size'])/topic['expected_size'],
                self.threshold)
        print("Ending test")


if __name__ == '__main__':
    unittest.main()
