from nav_msgs.msg import Odometry

class PoseHandler(object):

    def __init__(self, robotId, offsetX, offsetY):
        self.offsetX = offsetX
        self.offsetY = offsetY
        #Should have something to publish on too

    def callback(data):
        data.pose.pose.x = data.pose.pose.x + offsetX
        data.pose.pose.y = data.pose.pose.y + offsetY
        # publish on topic robots subscribe to
