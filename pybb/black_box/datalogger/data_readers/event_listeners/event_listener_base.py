from __future__ import print_function
import threading
import time
from abc import abstractmethod
from black_box.config.config_utils import ConfigUtils

class EventListenerBase(object):
    def __init__(self, name, event_type, max_frequency, data_logger):
        self.name = name
        self.event_type = event_type
        self.max_frequency = max_frequency
        self.sleep_time = 1.0/self.max_frequency
        self.data_logger = data_logger
        self.sub_thread = None
        self.logging = True

    def start(self):
        self.logging = True
        self.sub_thread = threading.Thread(target=self.run)
        self.sub_thread.start()

    def stop(self):
        self.logging = False
        print('['+self.name+'_event_listener] Stopping')
        self.sub_thread.join()
        self.sub_thread = None

    @abstractmethod
    def run(self):
        """Threaded method which should run continuously looking for events
        until self.logging is set to False
        :returns: None

        """
        while self.logging:
            time.sleep(self.sleep_time)
