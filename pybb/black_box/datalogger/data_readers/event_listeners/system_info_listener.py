from __future__ import print_function

import time
import rospy
from std_msgs.msg import String, Empty
from black_box.datalogger.data_readers.event_listeners.event_listener_base import EventListenerBase

class SystemInfoListener(EventListenerBase):

    """Listen for system info
    """

    def __init__(self, name, event_type, max_frequency, data_logger):
        super(SystemInfoListener, self).__init__(name, event_type, max_frequency, data_logger)
        self._logged = False
        self.ropod_get_info_pub = rospy.Publisher('/ropod_system_info/get_ropod_info',
                                                  Empty,
                                                  queue_size=1)
        self.ropod_info_sub = rospy.Subscriber('/ropod_system_info/ropod_info',
                                               String,
                                               self._get_ropod_info)

    def _get_data_on_startup(self):
        if self._logged:
            return None
        current_data = int(time.time())%10
        self._logged = True
        return {'time':current_data}

    def _get_ropod_info(self, msg):
        print(msg)
