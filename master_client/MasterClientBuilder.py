class StdMasterDirector():


class MasterBuilder():

    def __init__(self):
        self.masterclient = MasterClient()

    def buildNodes(self,nbr_of_nodes):
        self.masterclient.setNbrOfNodes(nbr_of_nodes)

    def buildUWBSensor(self):
        uwb = UWBHandler()
        self.masterclient.setPositionSensor(uwb)
        self.masterclient.setRotationSensor(uwb)
