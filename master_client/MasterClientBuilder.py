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
        self.builder.buildStdOptFunc()

class MasterBuilder():


    def __init__(self):
        self.com = ROScommunication()
        self.masterclient = MasterClient(self.com)

    def buildNodes(self,nbr_of_nodes):
        self.masterclient.setNbrOfNodes(nbr_of_nodes)

    def buildUWBSensor(self):
        uwb = UWBHandler(self.com)
        self.masterclient.setPositionSensor(uwb)
        self.masterclient.setRotationSensor(uwb)

    def buildKalman(self):
        self.masterclient.setFilter(Kalman())

    def buildStdOptFunc(self):
        self.masterclient.setOptFunc(OptFunc())

    def getClient(self):
        return self.masterclient
