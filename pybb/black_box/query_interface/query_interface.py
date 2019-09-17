import uuid
import time
from ropod.pyre_communicator.base_class import RopodPyre
from black_box.query_interface.db_interface import DBInterface

class BlackBoxQueryInterface(RopodPyre):
    '''An interface for querying a robotic black box

    @author -- Alex Mitrevski
    @contact aleksandar.mitrevski@h-brs.de

    '''
    def __init__(self, data_sources, black_box_id, groups, db_name='logs', db_port=27017):
        super(BlackBoxQueryInterface, self).__init__({
                'node_name': black_box_id + '_query_interface',
                'groups': groups,
                'message_types': list()})
        self.data_sources = data_sources
        self.black_box_id = black_box_id
        self.db_interface = DBInterface(db_name, db_port)
        self.start()

    def zyre_event_cb(self, zyre_msg):
        '''Listens to "SHOUT" and "WHISPER" messages and returns a response
        if the incoming message is a query message for this black box
        '''
        if zyre_msg.msg_type in ("SHOUT", "WHISPER"):
            response_msg = self.receive_msg_cb(zyre_msg.msg_content)
            if response_msg:
                self.whisper(response_msg, zyre_msg.peer_uuid)

    def receive_msg_cb(self, msg):
        '''Processes requests for variable and data black box queries;
        returns a dictionary representing a JSON response message

        Only listens to "DATA-QUERY" AND "VARIABLE-QUERY" messages for this black box;
        ignores all other messages (i.e. returns no value in such cases)

        @param msg a message in JSON format
        '''
        dict_msg = self.convert_zyre_msg_to_dict(msg)
        if dict_msg is None:
            return

        message_type = dict_msg['header']['type']
        if message_type == 'VARIABLE-QUERY':
            black_box_id = dict_msg['payload']['blackBoxId']
            if black_box_id != self.black_box_id:
                return

            variable_data = dict()
            for data_source in self.data_sources:
                variable_data[data_source] = self.db_interface.get_variables(data_source)
            response_msg = self.__get_response_msg_skeleton(message_type)
            response_msg['payload']['receiverId'] = dict_msg['payload']['senderId']
            response_msg['payload']['variableList'] = variable_data
            return response_msg
        elif message_type == 'DATA-QUERY':
            black_box_id = dict_msg['payload']['blackBoxId']
            if black_box_id != self.black_box_id:
                return

            start_time = float(dict_msg['payload']['startTime'])
            end_time = float(dict_msg['payload']['endTime'])
            variable_map = self.__get_variable_map(dict_msg['payload']['variables'])

            data = dict()
            for var_group_name, variable_names in variable_map.items():
                data_maps = self.db_interface.get_data(var_group_name, variable_names,
                                                       start_time, end_time)
                data.update(data_maps)

            response_msg = self.__get_response_msg_skeleton(message_type)
            response_msg['payload']['requestMsgId'] = dict_msg['header']['msgId']
            response_msg['payload']['receiverId'] = dict_msg['payload']['senderId']
            response_msg['payload']['dataList'] = data
            return response_msg
        elif message_type == 'LATEST-DATA-QUERY':
            black_box_id = dict_msg['payload']['blackBoxId']
            if black_box_id != self.black_box_id:
                return

            variable_map = self.__get_variable_map(dict_msg['payload']['variables'])
            data = dict()
            for var_group_name, variable_names in variable_map.items():
                data_maps = self.db_interface.get_latest_data(var_group_name, variable_names)
                data.update(data_maps)

            response_msg = self.__get_response_msg_skeleton(message_type)
            response_msg['payload']['requestMsgId'] = dict_msg['header']['msgId']
            response_msg['payload']['receiverId'] = dict_msg['payload']['senderId']
            response_msg['payload']['dataList'] = data
            return response_msg

    def __get_variable_map(self, variables):
        '''Returns a dictionary in which each key is a variable group name and the
        value is a list of variables corresponding to the group. Assumes that
        the given variables have the format "group_name/variable_name"
        (e.g. ros_cmd_vel/linear/x, where ros_cmd_val is the group name and
        linear/x is the variable name)

        Example:
        If varibles is the list ["ros_cmd_vel/linear/x", "ros_cmd_vel/linear/y",
        ros_cmd_vel/angular/z, "ros_pose/x", "ros_pose/y"], the resulting dictionary
        will have the format
        {
            "ros_cmd_vel": { ["linear/x", "linear/y", "angular/z"] },
            "ros_pose": { ["x", "y"] }
        }

        @param variables a list of variable names

        '''
        variable_map = dict()
        for variable_name in variables:
            slash_idx = variable_name.find('/')
            var_group_name = variable_name[0:slash_idx]
            var_name = variable_name[slash_idx+1:]

            if var_group_name not in variable_map:
                variable_map[var_group_name] = list()
            variable_map[var_group_name].append(var_name)
        return variable_map

    def __get_response_msg_skeleton(self, msg_type):
        '''Returns a dictionary representing a query response for the given message type.
        The dictionary has the following format:
        {
            "header":
            {
                "metamodel": "ropod-msg-schema.json",
                "type": msg_type,
                "msgId": message-uuid,
                "timestamp": current-time
            },
            "payload":
            {
                "receiverId": ""
            }
        }

        Keyword arguments:
        @param msg_type a string representing a message type

        '''
        response_msg = dict()
        response_msg['header'] = dict()
        response_msg['header']['metamodel'] = 'ropod-msg-schema.json'
        response_msg['header']['type'] = msg_type
        response_msg['header']['msgId'] = str(uuid.uuid4())
        response_msg['header']['timestamp'] = time.time()
        response_msg['payload'] = dict()
        response_msg['payload']['receiverId'] = ''
        return response_msg
