import importlib
from black_box.config.config_params import EventListenerParams
from black_box.config.config_utils import ConfigUtils
from black_box.datalogger.data_readers.event_listeners.event_listener_base import EventListenerBase
# from black_box.datalogger.data_readers.event_listeners.rosparam_listener import ROSParamListener

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
        self.listeners = []
        self.logging = False
        self.listener_classes = self.get_listener_classes()

    def start_logging(self):
        '''Initialise and start the listeners
        '''
        self.logging = True
        for Listener, listener_param in zip(self.listener_classes, self.config_params.listeners):
            self.listeners.append(
                    Listener(
                        listener_param.name, 
                        listener_param.event_type, 
                        listener_param.max_frequency, 
                        self.data_logger)
                    )
        for listener in self.listeners:
            listener.start()
        print('[EventReader] All Listener started')

    def stop_logging(self):
        '''Stops all listeners 
        '''
        self.logging = False
        print('[EventReader] Stopping all listeners')
        for listener in self.listeners:
            listener.stop()
        self.listeners = list()

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
