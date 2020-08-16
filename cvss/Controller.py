from abc import ABCMeta, abstractmethod

class Controller(object):
    __metaclass__ = ABCMeta

    def __init__(self):
        pass
    
    @abstractmethod
    def update(self, sensorData):
        '''
        Args:
            sensorData (cvss_msg_pb2.SensorData):
                Protocol buffer SensorData message object containing the most 
                recent simulated sensor values from the host machine.
        '''
