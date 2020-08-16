from abc import ABCMeta, abstractmethod

class ThreePiController(object):
    __metaclass__ = ABCMeta

    def __init__(self):
        pass

    def set_three_pi(self, three_pi):
        self.three_pi = three_pi
    
    @abstractmethod
    def update(self, sensorData):
        '''
        Args:
            sensorData (cvss_msg_pb2.SensorData):
                Protocol buffer SensorData message object containing the most 
                recent simulated sensor values from the host machine.
        '''
