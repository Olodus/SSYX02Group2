from enum import Enum

class Commands(Enum):
    getCoords = 0
    moveForward = 1
    rotate = 2

class MasterClient():

    def __init__(self):

    def run(self):

    def setPositionSensor(self,sensor):
        self.positionSensor = sensor

    def setRotationSensor(self,sensor):
        self.rotationSensor = sensor

    def setNbrOfNodes(self,nbr_of_nodes):
        self.nbr_of_nodes = nbr_of_nodes

    def setCommunication(com):
        self.com = com

    def setFilter(self,filter):
        self.filter = filter

    def setOptFunc(self,func):
        self.opt = func
