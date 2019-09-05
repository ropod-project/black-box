import importlib
import rospy
from multiprocessing import Process, Queue
from black_box.config.config_params import EventListenerParams
from black_box.config.config_utils import ConfigUtils
from black_box.datalogger.data_readers.event_listeners.event_listener_base import EventListenerBase

class EventReader(object):
    '''An interface for managing event listeners

    Constructor arguments:
    @param config_params -- an instance of black_box.config.config_params.RosParams
    @param max_frequency -- maximum frequency at which the node should be running
    @param data_logger -- a black_box.loggers.LoggerBase instance

    @author Dharmin B.

    '''
    def __init__(self, config_params, max_frequency, data_logger):
        self.config_params = config_params
        self.max_frequency = max_frequency
        self.data_logger = data_logger
        self.logging = False
        self.listener_classes = self.get_listener_classes()
        self.process = None
        self.stop_queue = Queue()

    def start_logging(self):
        '''Initialise and start the listeners
        '''
        self.logging = True
        self.process = Process(
                target=self.__create_event_listeners, 
                args=(self.stop_queue,),
                name='event_listener_process')
        self.process.start()

    def stop_logging(self):
        '''Stops all listeners 
        '''
        self.logging = False
        print('[EventReader] Stopping all listeners')
        if self.process is not None:
            self.stop_queue.put(True) # make stop_queue non empty
            self.process.join(timeout=3.0)
            if self.process.is_alive():
                self.process.terminate()
        print('[EventReader] Stopped all listeners')
        self.process = None

    def __create_event_listeners(self, stop_queue):
        """Create ros node and event listeners and wait till parent process 
        signals for shutdown. Once that signal is received (`stop_queue` is non
        empty), shutdown all listeners and shutdown ros node.

        :stop_queue: multiprocessing.Queue
        :returns: None

        """
        rospy.init_node('event_listener')
        listeners = []
        for Listener, listener_param in zip(self.listener_classes, self.config_params.listeners):
            listeners.append(
                    Listener(
                        listener_param.name, 
                        listener_param.event_type, 
                        listener_param.max_frequency, 
                        self.data_logger)
                    )
        for listener in listeners:
            listener.start()
        print('[EventReader] All Listener started')
        try:
            while stop_queue.empty():
                rospy.sleep(0.2)
        except Exception as e:
            print("[EventReader] Encountered error", str(e))
            # pass
        for listener in listeners:
            listener.stop()
        while not stop_queue.empty(): # make stop_queue empty again
            stop_queue.get()
        rospy.signal_shutdown("event listener logging was stopped.")
        print('[EventReader] process terminated')

    def get_listener_classes(self):
        """Import event listeners based on the config file.
        :returns: None

        """
        base_path = 'black_box.datalogger.data_readers.event_listeners.'
        listener_classes = []
        for listener_param in self.config_params.listeners:
            Listener = None

            try:
                src_code_file_name = listener_param.name + '_listener'
                module_name = base_path + src_code_file_name
                class_name = ''.join(x.title() for x in src_code_file_name.split('_'))
                Listener = getattr(importlib.import_module(module_name), class_name)
            except Exception as e:
                print('Encountered following Exception while importing', 
                        listener_param.name, '\n', str(e), '\n', 'Using EventListenerBase')
                Listener = EventListenerBase
            listener_classes.append(Listener)
        return listener_classes
