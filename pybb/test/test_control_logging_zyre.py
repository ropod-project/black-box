#!/usr/bin/env python

from __future__ import print_function
import time
import os.path
import json
import uuid
import sys

from ropod.pyre_communicator.base_class import RopodPyre
from ropod.utils.models import MessageFactory
from ropod.utils.uuid import generate_uuid
from black_box.config.config_file_reader import ConfigFileReader
from black_box_tools.db_utils import DBUtils

class TestPyreCommunicator(RopodPyre):

    """Test if starting and stopping of black box logging via zyre message is 
    possible or not.

    :groups: list of string (pyre groups)
    :black_box_id: string

    """

    def __init__(self, groups, black_box_id):
        super(TestPyreCommunicator, self).__init__(
                'bb_zyre_controller', groups, list(), verbose=True)
        self.black_box_id = black_box_id
        self.start()

        # self.send_request("BLACK-BOX_LOGGING_CMD")

    def send_request(self, msg_type, payload_dict=None):
        time.sleep(1)

        request_msg = dict()
        request_msg['header'] = dict()
        request_msg['header']['metamodel'] = 'ropod-black-box-logging-cmd-schema.json'
        request_msg['header']['type'] = msg_type
        request_msg['header']['msgId'] = str(uuid.uuid4())
        request_msg['header']['timestamp'] = time.time()
        request_msg['header']['blackBoxId'] = self.black_box_id

        request_msg['payload'] = dict()
        request_msg['payload']['senderId'] = generate_uuid()
        if payload_dict is not None :
            for key in payload_dict.keys() :
                request_msg['payload'][key] = payload_dict[key]

        request_msg = json.dumps(request_msg, indent=2, default=str)
        # print(request_msg)

        self.shout(request_msg)

    def receive_msg_cb(self, msg):
        '''Processes the incoming messages 

        :msg: string (a message in JSON format)

        '''
        dict_msg = self.convert_zyre_msg_to_dict(msg)
        if dict_msg is None:
            return

        if 'header' not in dict_msg or 'type' not in dict_msg['header']:
            return None
        message_type = dict_msg['header']['type']
        # print(dict_msg)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print('Usage: python3 test_control_logging_zyre.py [absolute-path-to-black-box-config-file]')
        sys.exit(1)
    bb_config_file = sys.argv[1]
    config_params = ConfigFileReader.load_config(bb_config_file)
    db_name = config_params.default.db_name
    db_port = 27017

    pyre_comm = TestPyreCommunicator(['ROPOD'], 'black_box_001')
    test_start_time = time.time()
    test_duration = 5
    print("Testing ... (for", test_duration, "seconds)")
    try:
        # First test case
        print("\n"*3, "Test 1", "\n"*3)
        pyre_comm.send_request("BLACK-BOX_LOGGING_CMD", {'cmd':"STOP"})
        while test_start_time + test_duration > time.time():
            newest_timestamp = DBUtils.get_db_newest_timestamp(db_name)
            if newest_timestamp < time.time():
                print("Stop test successfull")
            time.sleep(0.2)

        # Second test case
        print("\n"*3, "Test 2", "\n"*3)
        pyre_comm.send_request("BLACK-BOX_LOGGING_CMD", {'cmd':"START"})
        test_start_time = time.time()
        while test_start_time + test_duration > time.time():
            newest_timestamp = DBUtils.get_db_newest_timestamp(db_name)
            if newest_timestamp > test_start_time:
                print("Start test successfull")
            time.sleep(0.2)

        # Third test case
        print("\n"*3, "Test 3", "\n"*3)
        pyre_comm.send_request("BLACK-BOX_LOGGING_CMD", {'cmd':"STOP"})
        test_start_time = time.time()
        while test_start_time + test_duration > time.time():
            newest_timestamp = DBUtils.get_db_newest_timestamp(db_name)
            if newest_timestamp < time.time():
                print("Stop test successfull")
            time.sleep(0.2)
    except (KeyboardInterrupt, SystemExit):
        # print("Encountered following error", str(e))
        pass
        # print("Test FAILED", str(e))
    pyre_comm.shutdown()
