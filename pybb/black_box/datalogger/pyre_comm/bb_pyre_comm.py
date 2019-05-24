#!/usr/bin/env python

import time

from ropod.pyre_communicator.base_class import RopodPyre

class BlackBoxPyreCommunicator(RopodPyre):

    """Receive commands from other nodes 
    example: starting and stoping of logging action.

    :groups: list of string (pyre groups)
    :black_box_id: string

    """

    def __init__(self, groups, black_box_id):
        super(BlackBoxPyreCommunicator, self).__init__(
                'bb_pyre_comm'+black_box_id, groups, list(), verbose=True)
        self.logging = True
        self.black_box_id = black_box_id
        self.start()

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
        if message_type == "BLACK-BOX_LOGGING_CMD":
            if dict_msg['header'].get('blackBoxId', "") == self.black_box_id:
                print(dict_msg)
                if 'payload' not in dict_msg:
                    return None
                cmd = dict_msg['payload'].get('cmd', "")
                if cmd == 'START':
                    self.logging = True
                elif cmd == 'STOP':
                    self.logging = False
