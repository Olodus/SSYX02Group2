import rospy
from MasterClient import Commands

class ROScommunication():

    def __init__():

    def moveForward(length):
        rospy.wait_for_service(c)
        response = rospy.ServiceProxy(c,moveForward)

#    def serviceCom(command):
#        c = getServiceStringFromCommand(command)
#        rospy.wait_for_service(c)
#        response = rospy.ServiceProxy(c,GetCoords)
#
#    def getServiceStringFromCommand(command):
#        if command == Commands.getCoords:
#            return str('get_coord')
#        elif command == Commands.moveForward:
#            return str('moveRobot')
