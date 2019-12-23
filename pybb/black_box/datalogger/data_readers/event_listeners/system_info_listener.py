from __future__ import print_function

import rospy
import json
from std_msgs.msg import String, Empty
from black_box.datalogger.data_readers.event_listeners.event_listener_base import EventListenerBase

class SystemInfoListener(EventListenerBase):

    """Listen for system info
    """

    def __init__(self, name, event_type, max_frequency, data_logger):
        super(SystemInfoListener, self).__init__(name, event_type, max_frequency, data_logger)
        self._logged = False
        self._ropod_info = None
        self._wait_threshold = 2.0 # seconds
        self.ropod_get_info_pub = rospy.Publisher('/system_info/get_ropod_info',
                                                  Empty,
                                                  queue_size=1)
        self.ropod_info_sub = rospy.Subscriber('/system_info/ropod_info',
                                               String,
                                               self._get_ropod_info)

    def _get_data_on_startup(self):
        if self._logged:
            return None

        self.ropod_get_info_pub.publish(Empty())

        start_time = rospy.get_time()
        while self._ropod_info is None and rospy.get_time() < start_time + self._wait_threshold:
            rospy.sleep(0.1)

        if self._ropod_info is None:
            return None
        self._logged = True
        return {'ropod_info':self._ropod_info}

    def _get_ropod_info(self, msg):
        """ Subscriber for ropod_info. 
            {Assumption: msg.data is a json string}

        :msg: std_msgs.String
        :returns: None
        """
        self._ropod_info = json.loads(msg.data)
