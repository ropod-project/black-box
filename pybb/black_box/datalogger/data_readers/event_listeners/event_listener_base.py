from __future__ import print_function
import threading
import time
from abc import abstractmethod
from black_box.config.config_utils import ConfigUtils

class EventListenerBase(object):

    """Base class for event listerners
    
    @name -- string (name of the listener)
    @event_type -- string (['ON_CHANGE', 'ON_STARTUP'])
    @max_frequency -- float (maximum frequency at which the node should be running)
    @data_logger -- a black_box.loggers.LoggerBase instance

    """

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
        print('['+self.name+'_event_listener] Starting')
        self.sub_thread.start()

    def stop(self):
        self.logging = False
        print('['+self.name+'_event_listener] Stopping')
        self.sub_thread.join()
        self.sub_thread = None

    def run(self):
        """Threaded method which runs continuously looking for events until 
        self.logging is set to False. The looking for events has to be implemented
        by overrided method named like "_get_data_" + `event_type` (e.g. for event
        type 'ON_CHANGE' the method to be overriden should be named _get_data_on_change)
        This overriden methods should return a dict obj or None.

        :returns: None

        """
        while self.logging:
            data = self._get_data()
            if data is not None:
                self.data_logger.log_data(self.name, time.time(), data)
            time.sleep(self.sleep_time)

    def _get_data(self):
        """Calls a function named ("_get_data_" + `event_type`) to get data to
        be logged. If no such function exists, returns None otherwise returns 
        what that function returns.

        :returns: dict or None

        """
        func_name = "_get_data_" + self.event_type.lower()
        if not (hasattr(self, func_name) and callable(getattr(self, func_name))):
            print('['+self.name+'_event_listener] Invalid event_type. Not logging.')
            return None
        return getattr(self, func_name)()
