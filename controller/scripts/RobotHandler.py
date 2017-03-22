
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
		self.run()
        #self.x = data.pose.pose.x
		#self.y = data.pose.pose.y
		#self.vx = data.twist.linear.x

	def get_state(self):
		return {'state': self.state}

	def run(self):
		if executing:
			if self.mission = -1:
				print "Error: Robot " + str(self.id_nbr) + " is executing but has no mission defined."
			elif self.mission = 0:
				self.
		else:
			print "Robot " + str(self.id_nbr) + ": Is not doing anything"

	def is_ready(self):
		return {'robot_ready': not self.executing}

	def aim_at_point(self, req):
		if self.executing:
			error = "Error: Robot " + str(self.id_nbr) + ": Is already doing something"
			return {'succeded': False, 'error_msg': error}
		else:
			self.executing = True
			self.mission = 2
			self.mission_var = req.point
			msg = "Robot " + str(self.id_nbr) + ": Will aim itself towards given point"
			return {'succeded': True, 'error_msg': msg}



	def go_to_point(self, req):
		if self.executing:
			error = "Error: Robot " + str(self.id_nbr) + ": Is already doing something"
			return {'succeded': False, 'error_msg': error}
		else:
			self.executing = True
			self.mission = 0
			self.mission_var = req.point
			msg = "Robot " + str(self.id_nbr) + ": Will go to given point"
			return {'succeded': True, 'error_msg': msg}


	def follow_path(self, req):
		if self.executing:
			error = "Robot " + str(self.id_nbr) + ": Is already doing something"
			return {'succeded': False, 'error_msg': error}
		else:
			self.executing = True
			self.mission = 1
			self.mission_var = req.path
			msg = "Robot " + str(self.id_nbr) + ": Will follow given path"
			return {'succeded': True, 'error_msg': msg}

	# Is done when it reach the velocity
	def set_speed(self, req):
		if self.executing:
			error = "Robot " + str(self.id_nbr) + ": Can't set speed since already on a mission (use stop to abort mission)"
			return {'succeded': False, 'error_msg': error}
		else:
			self.executing = True
			self.moving = True
			self.mission = 3
			self.mission_var = req.speed
			msg = "Robot " + str(self.id_nbr) + ": Will set speed to " + str(req.speed)
			return {'succeded': True, 'error_msg': msg}

	def stop(self):
		if self.executing:
			error = "Robot " + str(self.id_nbr) + ": Stopped executing it's mission"
			self.mission = -1
			return {'succeded': True, 'error_msg': error}
		else:
			msg = "Robot " + str(self.id_nbr) + ": Can't stop since it wasn't executing a mission"
			return {'succeded': False, 'error_msg': msg}


	def set_acc(self, req):
		if self.executing:
			error = "Robot " + str(self.id_nbr) + ": Can't set acc since already on a mission (use stop to abort mission)"
			return {'succeded': False, 'error_msg': error}
		else:
			self.moving = True
			self.mission = 4
			self.mission_var = req.acc
			msg = "Robot " + str(self.id_nbr) + ": Will set acc to " + str(req.acc)
			return {'succeded': True, 'error_msg': msg}


	#Should only be used from inside this class. Never called from outside.
	#Yea I know I should have hid it in some way but I don't get how Python does components...
	def emulate_acc(self):
