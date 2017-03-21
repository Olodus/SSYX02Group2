
# actions
#  -1 : No action
#   0 : Go to Point
#   1 : Follow path
#   2 : Aim at point


class RobotHandler(object):
    def __init__(self, idNumber):
		self.idNumber = idNumber
		#self.twist = Twist()
		self.pub = rospy.Publisher("rosaria"+str(idNumber)+"/cmd_vel", Twist, queue_size=1)
		self.x = 0.0
		self.y = 0.0
		self.acc = 0.0
		self.vx = 0.0
		self.va = 0.0
		self.angle = 0.0

		self.executing = False
		self.moving = False
		self.action = -1
		self.cruising_speed = 0.3


	def run(self):
		if executing:
			if self.action = -1:
				print "Error: Robot " + str(idNumber) + " is executing but has no action defined."
			elif self.action = 0:
				self.
		else:
			print "Robot " + str(idNumber) + ": Is not doing anything"

	def is_ready(self):
		return not self.executing

    def update_state(self,data):
        self.x = data.pose.pose.x
		self.y = data.pose.pose.y
		self.vx = data.twist.linear.x

	def aim_at_point(self,point):
		if self.executing:
			error = "Robot " + str(idNumber) + ": Is already doing something"
			print error
			return error
		else:
			executing = True


	def go_to_point(point):
		if self.executing:
			error = "Error: Robot " + str(idNumber) + ": Is already doing something"
			print error
			return error
		elif self.moving:
			error = "Error: Robot " + str(idNumber) + ": Is already moving"
			print error
			return error
		else:
			executing = True

	def start_following_points(self, points):


	def follow_path(self,points):
		if self.executing:
			error = "Robot " + str(idNumber) + ": Is already doing something"
			print error
			return error
		else:
			executing = True
			print "Robot " + str(idNumber) + ": Following given path

	# Is done when it reach the velocity
	def move_forward_v(self,velocity):
		if self.executing:
			error = "Robot " + str(idNumber) + ": Is already doing something"
			print error
			return error
		else:
			print "Robot " + str(idNumber) + ": Moving forward with speed: " + str(velocity)
			self.moving = True

	def stopping():
		if self.executing:
			error = "Robot " + str(idNumber) + ": Is already doing something"
			print error
			return error
		else:
			print "Robot " + str(idNumber) + ": Stopping"


	def move_forward_a(self,acc):
		self.acc = acc

	def abort_action(self):
		print "Robot " + str(idNumber) + ": Aborting action"
		self.executing = False
		self.s

	#Should only be used from inside this class. Never called from outside.
	#Yea I know I should have hid it in some way but I don't get how Python does components...
	def emulate_acc(self):
        
