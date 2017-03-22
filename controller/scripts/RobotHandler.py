
# Missions
#  -1 : No mission
#   0 : Go to Point
#   1 : Follow path
#   2 : Aim at point
#	3 : Set speed
#	4 : Set acceleration


class RobotHandler(object):
    def __init__(self, id_nbr):
        rospy.init_node('robot'+str(id_nbr))
		self.id_nbr = id_nbr
		self.pub = rospy.Publisher("rosaria"+str(id_nbr)+"/cmd_vel", Twist, queue_size=1)

		# State
		self.state = Odometry()
		self.executing = False
		self.moving = False

		# Mission
		self.mission = -1
		self.mission_var = None

		# Constants
		self.cruising_speed = 0.3

		##### Creates all services #####

		# Creating 'is_ready' service
		a = rospy.Service('robot'+str(id_nbr)+'_is_ready', Bool, self.is_ready)

		# Creating 'aim_at_point' service
		a = rospy.Service('robot'+str(id_nbr)+'_aim_at_point', Bool, self.aim_at_point)

		# Creating 'go_to_point' service
		a = rospy.Service('robot'+str(id_nbr)+'_go_to_point', Bool, self.go_to_point)

		# Creating 'follow_path' service
		a = rospy.Service('robot'+str(id_nbr)+'_follow_path', FollowPath, self.follow_path)

		# Creating 'set_speed' service
		a = rospy.Service('robot'+str(id_nbr)+'_set_speed', Bool, self.set_speed)

		# Creating 'stop' service
		a = rospy.Service('robot'+str(id_nbr)+'_stop', Bool, self.stop)

		# Creating 'set_acc' service
		a = rospy.Service('robot'+str(id_nbr)+'_set_acc', Bool, self.set_acc)

		rospy.spin()



    def update_state(self,data):
		self.state = data

        #self.x = data.pose.pose.x
		#self.y = data.pose.pose.y
		#self.vx = data.twist.linear.x

	def get_state(self):


	def run(self):
		if executing:
			if self.mission = -1:
				print "Error: Robot " + str(self.id_nbr) + " is executing but has no mission defined."
			elif self.mission = 0:
				self.
		else:
			print "Robot " + str(self.id_nbr) + ": Is not doing anything"

	def is_ready(self):
		return not self.executing

	def aim_at_point(self,point):
		if self.executing:
			error = "Robot " + str(self.id_nbr) + ": Is already doing something"
			print error
			return error
		else:
			executing = True


	def go_to_point(point):
		if self.executing:
			error = "Error: Robot " + str(self.id_nbr) + ": Is already doing something"
			print error
			return error
		elif self.moving:
			error = "Error: Robot " + str(self.id_nbr) + ": Is already moving"
			print error
			return error
		else:
			executing = True


	def follow_path(self,points):
		if self.executing:
			error = "Robot " + str(self.id_nbr) + ": Is already doing something"
			print error
			return error
		else:
			executing = True
			print "Robot " + str(self.id_nbr) + ": Following given path

	# Is done when it reach the velocity
	def set_speed(self,velocity):
		if self.executing:
			error = "Robot " + str(self.id_nbr) + ": Is already doing something"
			print error
			return error
		else:
			print "Robot " + str(self.id_nbr) + ": Moving forward with speed: " + str(velocity)
			self.moving = True

	def stop():
		if self.executing:
			error = "Robot " + str(self.id_nbr) + ": Is already doing something"
			print error
			return error
		else:
			print "Robot " + str(self.id_nbr) + ": Stopping"


	def set_acc(self,acc):
		self.acc = acc


	#Should only be used from inside this class. Never called from outside.
	#Yea I know I should have hid it in some way but I don't get how Python does components...
	def emulate_acc(self):
