from __future__ import print_function

import time
import json
import uuid

from ropod.utils.uuid import generate_uuid
from ropod.pyre_communicator.base_class import RopodPyre

from black_box.config.config_utils import ConfigUtils

class ZyrePublisher(RopodPyre):

    """Publish a zyre message for a definite number of times with a certain frequency.

    :msg_type: string (zyre message type)
    :groups: list of strings (list of zyre groups this node should be part of)
    :keyword arguments:
    :num_of_msgs: int (num of messages that needs to be published)
    :max_frequency: float (how many msgs should be be published in a second)
    """

    def __init__(self, msg_type, groups, *args, **kwargs):
        super(ZyrePublisher, self).__init__(
                'zyre_publisher_automatic_test_'+msg_type, groups, list(), verbose=False, acknowledge=False)
        self.msg_type = msg_type
        self.num_of_msgs = kwargs.get('num_of_msgs', 10)
        self.sleep_time = 1.0/kwargs.get('max_frequency', 10)
        self.publishing = False
        self.start()

    def start_publishing(self):
        '''publish empty messages.
        '''
        self.publishing = True
        print('Started', self.msg_type, 'zyre publisher')
        for i in range(self.num_of_msgs):
            if not self.publishing:
                break
            self._send_request(self.msg_type)
            time.sleep(self.sleep_time)
        self.publishing = False
        print('before shutdown')
        self.shutdown()
        print('after shutdown')
        
    def _send_request(self, msg_type, payload_dict=None):
        request_msg = dict()
        request_msg['header'] = dict()
        request_msg['header']['metamodel'] = 'ropod-msg-schema.json'
        request_msg['header']['type'] = msg_type
        request_msg['header']['msgId'] = str(uuid.uuid4())
        request_msg['header']['timestamp'] = time.time()

        request_msg['payload'] = dict()
        request_msg['payload']['senderId'] = generate_uuid()
        if payload_dict is not None :
            for key in payload_dict.keys() :
                request_msg['payload'][key] = payload_dict[key]

        # print(json.dumps(request_msg, indent=2, default=str))

        self.shout(request_msg)

if __name__ == "__main__":
    zyre_pub = ZyrePublisher('CMD', ['ROPOD'], num_of_msgs=100)

    print('publishing')
    zyre_pub.start_publishing()

    print('published')
