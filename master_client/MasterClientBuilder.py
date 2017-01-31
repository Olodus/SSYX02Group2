import UWBHandler
import MasterClient
import Kalman

class StdMasterDirector():
    
    def __init__(self,builder):
        self.builder = builder

    def construct(self):
        self.builder = MasterBuilder()

        self.builder.buildNodes(2)
        self.builder.buildUWBSensor()

class MasterBuilder():

    def __init__(self):
        self.masterclient = MasterClient()

    def buildNodes(self,nbr_of_nodes):
        self.masterclient.setNbrOfNodes(nbr_of_nodes)

    def buildUWBSensor(self):
        uwb = UWBHandler()
        self.masterclient.setPositionSensor(uwb)
        self.masterclient.setRotationSensor(uwb)

    def buildKalman(self):
        self.masterclient.setFilter(Kalman())

    def getClient(self):
        return self.masterclient
