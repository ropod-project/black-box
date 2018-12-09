from abc import abstractmethod

class DataReaderBase(object):
    def __init__(self, data_logger):
        self.data_logger = data_logger

    @abstractmethod
    def start(self):
        pass

    @abstractmethod
    def stop(self):
        pass
