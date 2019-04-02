from __future__ import print_function

import time
import rospy
from black_box.datalogger.data_readers.event_listeners.event_listener_base import EventListenerBase

class TestListener(EventListenerBase):

    """Test listener
    
    @name -- string (name of the listener)
    @event_type -- string (['CHANGE'])
    """

    def __init__(self, name, event_type, max_frequency, data_logger):
        super(TestListener, self).__init__(name, event_type, max_frequency, data_logger)
        self._last_logged_msg = None

    def run(self):
        time.sleep(2)
        while self.logging:
            keys = rospy.get_param_names()
            # keys = rospy.get_published_topics()
            print(keys)
            if self._last_logged_msg is None or \
                    self._last_logged_msg + 1 < time.time():
                self.data_logger.log_data(
                        self.name, 
                        time.time(), 
                        {'time':time.time()})
                self._last_logged_msg = time.time()
            time.sleep(self.sleep_time)
