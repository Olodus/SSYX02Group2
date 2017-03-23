import rospy

class KalmanFilter(object):
    def __init__(self, robot_id):
        # Does this need to be a node? Maybe... maybe not... ?
        rospy.init_node('Filter'+str(robot_id))

        self.pub = rospy.Publisher("Filter"+str(robot_id)+"/prediction", Odometry, queue_size=1)
        self.prediction = Odometry()

    def new_measurement(self, data):
        self.prediction = data
        
        self.pub.publish(self.prediction)
