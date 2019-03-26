#!/usr/bin/env python

from __future__ import print_function
import time
import os.path
import json
import uuid

from ropod.pyre_communicator.base_class import RopodPyre
from ropod.utils.models import MessageFactory
from ropod.utils.uuid import generate_uuid

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
        print(request_msg)

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
    pyre_comm = TestPyreCommunicator(['ROPOD'], 'black_box_001')
    test_start_time = time.time()
    test_duration = 10
    print("Testing ... (for", test_duration, "seconds)")
    try:
        pyre_comm.send_request("BLACK-BOX_LOGGING_CMD", {'cmd':"STOP"})
        while test_start_time + test_duration > time.time():
            time.sleep(0.1)

        pyre_comm.send_request("BLACK-BOX_LOGGING_CMD", {'cmd':"START"})
        test_start_time = time.time()
        while test_start_time + test_duration > time.time():
            time.sleep(0.1)
        pyre_comm.send_request("BLACK-BOX_LOGGING_CMD", {'cmd':"STOP"})
        # print("Test PASSED")
    except Exception as e:
        print("Encountered following error", str(e))
        pass
        # print("Test FAILED", str(e))
    pyre_comm.shutdown()
