
# Missions
#  -1 : No mission
#   0 : Go to Point
#   1 : Follow path
#   2 : Aim at point
#	3 : Set speed
#	4 : Set acceleration
#	5 : Stop


class RobotHandler(object):
    def __init__(self, id_nbr):
        rospy.init_node('robot'+str(id_nbr))
		self.id_nbr = id_nbr
		self.name = 'robot'+str(id_nbr) #TODO replace all places name could be used instead
		self.pub = rospy.Publisher("rosaria"+str(id_nbr)+"/cmd_vel", Twist, queue_size=1)
		self.twist = Twist()

		# State
		self.state = Odometry()
		self.executing = False
		self.moving = False

		# Mission
		self.mission = -1
		self.aim_point = Point()
		self.angle_to_point = 0.0
		self.go_point = Point()
		self.length_to_point = 0.0
		self.path = Path()
		self.path_index = 0
		self.desired_speed = 0.0
		self.acc = 0.0

		# Constants
		self.cruising_speed = 0.3
		self.max_speed = 1.0

		##### Creates all services #####

		# Creating 'is_ready' service
		a = rospy.Service('robot'+str(id_nbr)+'_is_ready', IsReady, self.is_ready)

		# Creating 'aim_at_point' service
		a = rospy.Service('robot'+str(id_nbr)+'_aim_at_point', AimAtPoint, self.aim_at_point)

		# Creating 'go_to_point' service
		a = rospy.Service('robot'+str(id_nbr)+'_go_to_point', GoToPoint, self.go_to_point)

		# Creating 'follow_path' service
		a = rospy.Service('robot'+str(id_nbr)+'_follow_path', FollowPath, self.follow_path)

		# Creating 'set_speed' service
		a = rospy.Service('robot'+str(id_nbr)+'_set_speed', SetSpeed, self.set_speed)

		# Creating 'stop' service
		a = rospy.Service('robot'+str(id_nbr)+'_stop', Stop, self.stop)

		# Creating 'set_acc' service
		a = rospy.Service('robot'+str(id_nbr)+'_set_acc', SetAcc, self.set_acc)

		rospy.spin()



    def update_state(self,data):
		self.state = data
		self.run()
		self.publish_twist()
		self.check_if_mission_done()

	def run(self):
		self.twist = Twist()
		if self.mission = 0:
			do_go_to_point()
		elif self.mission = 1:
			do_follow_path()
		elif self.mission = 2:
			do_aim_at_point()
		elif self.mission = 3:
			do_set_speed()
		elif self.mission = 4:
			do_set_acc()
		elif self.mission = 5:
			do_stop()

	def publish_twist(self):
		self.pub.publish(self.twist)

	def check_if_mission_done(self):
		if self.mission = 0:
			# Go to point
			if self.length_to_point <= 0.1:
				self.executing = False
				self.mission = -1
		elif self.mission = 1:
			# Follow path
			if self.path_index = self.path.length:
				length =
				if length <= 0.1:
					self.executing = False
					self.mission = -1
		elif self.mission = 2:
			# Aim at point
			if self.angle_to_point <= 0.05:
				self.executing = False
				self.mission = -1
		elif self.mission = 3:
			# Set speed
			if self.state.twist.twist.linear.x - desired_speed >= 0.001:
				self.executing = False
				self.mission = -1
				self.moving = True
		elif self.mission = 4:
			# Set acc
			# dont need to do something right
			self.moving = True
		elif self.mission = 5:
			# Stop
			if self.state.twist.twist.linear.x <= 0.0001:
				self.executing = False
				self.mission = -1


	def get_state(self):
		return {'state': self.state}

	def is_ready(self):
		return {'robot_ready': not self.executing}

	def aim_at_point(self, req):
		if self.executing:
			error = "Error: Robot " + str(self.id_nbr) + ": Is already doing something"
			return {'succeded': False, 'error_msg': error}
		else:
			self.executing = True
			self.mission = 2
			self.aim_point = req.point
			msg = "Robot " + str(self.id_nbr) + ": Will aim itself towards given point"
			return {'succeded': True, 'error_msg': msg}



	def go_to_point(self, req):
		if self.executing:
			error = "Error: Robot " + str(self.id_nbr) + ": Is already doing something"
			return {'succeded': False, 'error_msg': error}
		else:
			self.executing = True
			self.mission = 0
			self.go_point = req.point
			msg = "Robot " + str(self.id_nbr) + ": Will go to given point"
			return {'succeded': True, 'error_msg': msg}


	def follow_path(self, req):
		if self.executing:
			error = "Robot " + str(self.id_nbr) + ": Is already doing something"
			return {'succeded': False, 'error_msg': error}
		else:
			self.executing = True
			self.mission = 1
			self.path = req.path
			self.path_index = 0
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
			self.desired_speed = req.speed
			msg = "Robot " + str(self.id_nbr) + ": Will set speed to " + str(req.speed)
			return {'succeded': True, 'error_msg': msg}

	def stop(self):
		if self.executing:
			error = "Robot " + str(self.id_nbr) + ": Stopped executing it's mission"
			self.mission = 5
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
			self.acc = req.acc
			msg = "Robot " + str(self.id_nbr) + ": Will set acc to " + str(req.acc)
			return {'succeded': True, 'error_msg': msg}


	def do_go_to_point(self):
		x = self.state.pose.pose.position.x
		y = self.state.pose.pose.position.y
		robot_angle = self.state.pose.pose.orientation #TODO transform from quaternion
		pointx = self.go_point.x
		pointy = self.go_point.y
		angle = math.arctan2(x-pointx,y-pointy)
		self.length_to_point =
		length = self.length_to_point
		if angle-robot_angle >= 0.1
			self.aim_point = self.go_point
			do_aim_at_point()
		elif length >= 1.0:
			desired_speed = 0.3
			do_set_speed()
		elif length >= 0.5:
			desired_speed = 0.2
			do_set_speed()
			small_steer()
		elif length >= 0.25:
			desired_speed = 0.1
			do_set_speed()
		elif length >= 0.1:
			do_stop()

	def do_follow_path(self):


	def do_aim_at_point(self):

	def do_set_speed(self):
		self.twist.linear.x = desired_speed

	def do_set_acc(self):

	def do_stop(self):
		self.twist.linear.x = 0.0


	#Should only be used from inside this class. Never called from outside.
	#Yea I know I should have hid it in some way but I don't get how Python does components...
	def emulate_acc(self):
		#TODO Create acc implementation that handles accual time not just 0.1 sec
		curr_speed = self.state.twist.twist.linear.x
		desired_speed = curr_speed + self.acc*0.1
		do_set_speed()

	def small_steer(self):
