import rospy
from MasterClient import Commands

class ROScommunication():

    def __init__():

    def moveForward(robotId,length):
        pub = rospy.Publisher("ros" .. robotId .. "/cmd_vel", Twist, queue_size=10)

        # Sleeping for 0.01s to ensure ready for movement
        twist = Twist()
        pub.publish(twist)
        rospy.loginfo("Sleeping for 10ms to make sure it's stopped.")
        rospy.sleep(0.01);

        # Update twistmessage with length to move
        twist.linear.x = (req.length)
        rospy.loginfo("Moving robot.")
        pub.publish(twist)
        rospy.sleep(1)

        # Ensure movement stop
        rospy.loginfo("Stopping.")
        twist = Twist()
        pub.publish(twist)
        rospy.sleep(0.01)



        #rospy.wait_for_service(c)
        #response = rospy.ServiceProxy(c,moveForward)

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
