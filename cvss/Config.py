import json

class Config:
    '''
    Config is a class used to import variables from a JSON config file at runtime.
    
    Attributes:
        serverIP (str): IP address of the host computer running the CVSensorSimulator application. 
        tagID (int): Tag ID number of the AprilTag on this robot. 
        label (str): This robot's display name.
        debug_mode (boolean): Flag enabling debug mode. Debug mode removes calls to Arduino serial port, allows testing in desktop environment. 
    '''

    def __init__(self, filepath):
        self.serverIP = "192.168.1.0"
        self.tagID = 1
        self.label = "Robot1"
        self.parse_configs(filepath)

    def parse_configs(self, filepath):
        with open(filepath, 'r') as jsonFile:
            config = json.load(jsonFile)

        self.serverIP = config['serverIP']
        self.port = config['port']
        self.tagID = config['tagID']
        self.label = config['label']
        self.env = config['env']
        self.robotRadius = config['robotRadius']