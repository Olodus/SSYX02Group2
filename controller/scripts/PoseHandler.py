from nav_msgs.msg import Odometry

class PoseHandler(object):

    def __init__(self, robot_id, offsetX, offsetY):
        rospy.init_node('Sensor'+str(robot_id))
        self.offsetX = offsetX
        self.offsetY = offsetY
        self.pub = rospy.Publisher("Sensor"+str(robot_id)+"/measurement", Odometry, queue_size=1)

    def measure(self, data):
        data.pose.pose.x = data.pose.pose.x + self.offsetX
        data.pose.pose.y = data.pose.pose.y + self.offsetY
        self.pub.publish(data)
