from __future__ import print_function

import time
import subprocess
from black_box.datalogger.data_readers.event_listeners.event_listener_base import EventListenerBase

class RosparamListener(EventListenerBase):

    """Listen for ros param changes
    
    @name -- string (name of the listener)
    @event_type -- string (['ON_CHANGE'])
    """

    def __init__(self, name, event_type, max_frequency, data_logger):
        super(RosparamListener, self).__init__(name, event_type, max_frequency, data_logger)
        self._last_logged_ros_param = None

    def _get_data_on_change(self):
        current_ros_param = self.get_current_ros_params_as_dict()
        if self._last_logged_ros_param is None or self._last_logged_ros_param != current_ros_param:
            self._last_logged_ros_param = current_ros_param
            return {'rosparam':current_ros_param}
        return None

    def get_current_ros_params_as_dict(self):
        """Get all ros parameters and return them as a dictionary object
        :returns: dict

        """
        ans = subprocess.check_output(['rosparam', 'list'])
        keys = ans.decode('utf-8').split('\n')[:-1]

        ros_params = dict()
        for key in keys:
            something = subprocess.check_output(['rosparam', 'get', key])
            value = something.decode('utf-8').strip()
            if value[0] == value[-1] == '\'':
                value = value[1:-1].strip()
            ros_params[key] = value

        return ros_params
