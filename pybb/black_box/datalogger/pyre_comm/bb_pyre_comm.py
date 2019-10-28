#!/usr/bin/env python

import datetime
from ropod.pyre_communicator.base_class import RopodPyre
from black_box_tools.db_utils import DBUtils

class BlackBoxPyreCommunicator(RopodPyre):

    """Receive commands from other nodes
    example: starting and stoping of logging action.

    :groups: list of string (pyre groups)
    :black_box_id: string

    """

    def __init__(self, groups, config_params, logger):
        self.bb_config_params = config_params
        self.black_box_id = config_params.zyre.node_name
        self.data_logger = logger
        self.logging = True
        super(BlackBoxPyreCommunicator, self).__init__({
            'node_name': 'bb_pyre_comm' + self.black_box_id,
            'groups': groups,
            'message_types': list()})
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
        if message_type == "BLACK-BOX-LOGGING-CMD":
            if dict_msg['header'].get('blackBoxId', "") == self.black_box_id:
                if 'payload' not in dict_msg:
                    return None
                cmd = dict_msg['payload'].get('cmd', "")
                if cmd == 'START':
                    self.logging = True
                elif cmd == 'STOP' or cmd == 'PAUSE':
                    self.logging = False
        elif message_type == 'ROBOT-EXPERIMENT-REQUEST':
            if dict_msg['header'].get('blackBoxId', "") != self.black_box_id:
                return None
            self.dump_and_reinit_bbdb('old_data')
            self.logging = True
        elif message_type == 'ROBOT-EXPERIMENT-FEEDBACK':
            if dict_msg['header'].get('blackBoxId', "") != self.black_box_id:
                return None
            if 'payload' not in dict_msg:
                return None
            experiment_type = dict_msg['payload']['experimentType']
            self.dump_and_reinit_bbdb(experiment_type)
            self.logging = False
        elif message_type == 'ROBOT-EXPERIMENT-CANCEL':
            if dict_msg['header'].get('blackBoxId', "") != self.black_box_id:
                return None
            self.dump_and_reinit_bbdb('cancelled_experiment')
            self.logging = False

    def dump_and_reinit_bbdb(self, data_descriptor):
        '''Dumps the current black box database in self.bb_config_params.default.db_export_dir,
        drops the database, and then reinitialises a new black box database
        with the black box metadata.

        Keyword arguments:
        data_descriptor: str -- descriptive name of the directory in which
                                the data is be exported

        '''
        dump_dir = '{0}/{1}_{2}'.format(self.bb_config_params.default.db_export_dir,
                                        data_descriptor,
                                        datetime.datetime.now().isoformat())

        # we dump the black box database
        print('Dumping database {0} to {1}'.format(self.data_logger.db_name,
                                                   dump_dir))
        DBUtils.dump_db(db_name=self.data_logger.db_name,
                        data_dir=dump_dir,
                        delete_db=True)

        # we reinitialise the black box database
        # by writing the metadata
        print('Reinitialising database {0}'.format(self.data_logger.db_name))
        self.data_logger.write_metadata(self.bb_config_params)
        print('Database {0} reinitialised'.format(self.data_logger.db_name))
