from __future__ import print_function

import time
import rospy
from black_box.datalogger.data_readers.event_listeners.event_listener_base import EventListenerBase

class TestListener(EventListenerBase):

    """Test listener
    
    @name -- string (name of the listener)
    @event_type -- string (['ON_CHANGE', 'ON_STARTUP'])
    """

    def __init__(self, name, event_type, max_frequency, data_logger):
        super(TestListener, self).__init__(name, event_type, max_frequency, data_logger)
        self._last_logged_msg = None
        self._logged = False

    def _get_data_on_change(self):
        current_data = int(time.time())%10
        if self._last_logged_msg is None or self._last_logged_msg != current_data:
            self._last_logged_msg = current_data
            return {'time':current_data}
        return None
    
    def _get_data_on_startup(self):
        if self._logged:
            return None
        current_data = int(time.time())%10
        self._logged = True
        return {'time':current_data}
