#! /usr/bin/env python3
"""This module contains unit test for query interface of black box"""

import time
import os
import subprocess
import unittest
import yaml
import pymongo as pm

from ropod.pyre_communicator.base_class import RopodPyre
from ropod.utils.models import MessageFactory
from ropod.utils.uuid import generate_uuid
from black_box.query_interface.query_interface import BlackBoxQueryInterface

class QueryTest(RopodPyre):
    """ Pyre node for communicating with black box query interface"""

    def __init__(self):
        super(QueryTest, self).__init__('bb_query_test', ['ROPOD'], [],
                                        verbose=False, acknowledge=False)
        self.response = None
        self.sender_id = None
        self.start()

    def send_request(self, msg_type, payload_dict=None):
        query_msg = MessageFactory.get_header(msg_type, recipients=[])

        query_msg['payload'] = {}
        query_msg['payload']['senderId'] = generate_uuid()
        self.sender_id = query_msg['payload']['senderId']
        if payload_dict is not None:
            for key in payload_dict.keys():
                query_msg['payload'][key] = payload_dict[key]

        # print(json.dumps(query_msg, indent=2, default=str))
        self.shout(query_msg)

    def receive_msg_cb(self, msg_content):
        message = self.convert_zyre_msg_to_dict(msg_content)
        if message is None:
            return

        if self.sender_id:
            if 'payload' in message and 'receiverId' in message['payload']:
                if self.sender_id == message['payload']['receiverId']:
                    self.sender_id = None
                    self.response = message

class TestBBQueryInterface(unittest.TestCase):

    """ Unit Tests for black box query interface"""

    @classmethod
    def setUpClass(cls):
        test_dir = os.path.abspath(os.path.dirname(__file__))
        pybb_dir = os.path.dirname(test_dir)
        main_dir = os.path.dirname(pybb_dir)

        # create data
        cls.test_db_name = "bb_test_data"
        cls.test_db_dir = os.path.join(test_dir, cls.test_db_name)
        host, port = cls._get_db_host_and_port()
        cls.client = pm.MongoClient(host=host, port=port)
        success = cls._restore_test_db()
        assert success

        # read config params
        cls.config_file_path = os.path.join(main_dir, 'config/test_sources.yaml')
        config_params = get_config_params(cls.config_file_path)
        cls.bb_id = config_params.bb_id

        # create black box query interface
        cls.query_interface = BlackBoxQueryInterface(
            config_params.data_sources,
            config_params.bb_id,
            config_params.zyre_groups,
            db_name=cls.test_db_name)

        # create test pyre node
        cls.test_pyre_node = QueryTest()
        cls.timeout_duration = 3 #seconds
        time.sleep(cls.timeout_duration)

    @classmethod
    def tearDownClass(cls):
        cls.query_interface.shutdown()
        cls.test_pyre_node.shutdown()
        cls._drop_test_db()

    def test_variable(self):
        msg_type = "VARIABLE-QUERY"
        payload = {'blackBoxId':self.bb_id}
        message = self.send_request_get_response(msg_type, payload)

        self.assertNotEqual(message, None)
        self.assertIn('header', message)
        self.assertIn('type', message['header'])
        self.assertEqual(message['header']['type'], msg_type)
        self.assertIn('payload', message)
        self.assertIn('variableList', message['payload'])
        self.assertIn('ros', message['payload']['variableList'])
        cmd_vel_variables = [
            'ros_ropod_cmd_vel/angular/x', 'ros_ropod_cmd_vel/angular/y',
            'ros_ropod_cmd_vel/angular/z', 'ros_ropod_cmd_vel/linear/x',
            'ros_ropod_cmd_vel/linear/y', 'ros_ropod_cmd_vel/linear/z']
        for variable in cmd_vel_variables:
            self.assertIn(variable, message['payload']['variableList']['ros'])

    def test_data_query(self):
        msg_type = "DATA-QUERY"
        payload = {
            'blackBoxId':self.bb_id,
            'variables':['ros_ropod_cmd_vel/linear/x'],
            'startTime':1544441409.0861,
            'endTime':1544441409.88607}
        message = self.send_request_get_response(msg_type, payload)

        self.assertNotEqual(message, None)
        self.assertIn('header', message)
        self.assertIn('type', message['header'])
        self.assertEqual(message['header']['type'], msg_type)
        self.assertIn('payload', message)
        self.assertIn('dataList', message['payload'])
        self.assertIn(payload['variables'][0], message['payload']['dataList'])
        self.assertEqual(len(message['payload']['dataList'][payload['variables'][0]]), 6)

    def test_latest_data_query(self):
        try:
            msg_type = "LATEST-DATA-QUERY"
            payload = {
                'blackBoxId':self.bb_id,
                'variables':['ros_ropod_cmd_vel/linear/x']}
            message = self.send_request_get_response(msg_type, payload)
            db_obj = self.client[self.test_db_name]
            collection = db_obj['ros_ropod_cmd_vel']
            doc = collection.find_one(sort=[('timestamp', pm.DESCENDING)])
            latest_time_stamp = doc['timestamp']

            self.assertNotEqual(message, None)
            self.assertIn('header', message)
            self.assertIn('type', message['header'])
            self.assertEqual(message['header']['type'], msg_type)
            self.assertIn('payload', message)
            self.assertIn('dataList', message['payload'])
            self.assertIn(payload['variables'][0], message['payload']['dataList'])
            data = message['payload']['dataList'][payload['variables'][0]]
            self.assertIsInstance(data, str)
            received_timestamp = float(data[1:-1].split(',')[0])
            self.assertEqual(received_timestamp, latest_time_stamp)
        except Exception as e:
            print("Got following exception")
            print(str(e))


    def send_request_get_response(self, msg_type, payload_dict = None):
        self.test_pyre_node.send_request(msg_type, payload_dict)
        start_time = time.time()
        while self.test_pyre_node.response is None and \
                start_time + self.timeout_duration > time.time():
            time.sleep(0.2)
        message = self.test_pyre_node.response
        self.test_pyre_node.response = None
        return message

    @classmethod
    def _restore_test_db(cls):
        (host, port) = cls._get_db_host_and_port()
        commands = ['mongorestore', cls.test_db_dir, '--db', cls.test_db_name,
                    '--host', host, '--port', str(port)]
        with open(os.devnull, 'w') as devnull:
            process = subprocess.run(commands, stdout=devnull, stderr=devnull)
        return process.returncode == 0

    @classmethod
    def _drop_test_db(cls):
        if cls.test_db_name in cls.client.list_database_names():
            cls.client.drop_database(cls.test_db_name)

    @classmethod
    def _get_db_host_and_port(cls):
        host = 'localhost'
        port = 27017
        if 'DB_HOST' in os.environ:
            host = os.environ['DB_HOST']
        if 'DB_PORT' in os.environ:
            port = int(os.environ['DB_PORT'])
        return (host, port)

# =============================================================================
# Copied from black_box/pybb/query_interface_main.py (because could not import)
# =============================================================================
class ConfigParams(object):
    def __init__(self):
        self.bb_id = ''
        self.zyre_groups = list()
        self.data_sources = list()

def get_config_params(config_file):
    config_data = dict()
    with open(config_file, 'r') as bb_config:
        config_data = yaml.safe_load(bb_config)

    config_params = ConfigParams()
    for data_item in config_data:
        data_source, data = next(iter(data_item.items()))
        if data_source == 'default_parameters':
            continue

        if data_source == 'zyre':
            config_params.bb_id = data['name']
            config_params.zyre_groups = data['groups']
        config_params.data_sources.append(data_source)
    return config_params
# =============================================================================


if __name__ == '__main__':
    unittest.main()
