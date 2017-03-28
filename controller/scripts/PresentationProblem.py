from geometry_msgs.msg import Point

class PresentationProblem(object):
    def __init__(self):
        self.nbr_of_robots = 1
        self.start_position = [Point()]
        self.robots = []

    def numberOfRobots(self):
        return self.nbr_of_robots

    def get_start_positions(self):
        return self.start_position

#    def wait_til_ready(self):
        # loop through and see if all robots are ready

    def run(self, robots):
        if not len(robots)==self.nbr_of_robots:
            print "Gave problem wrong number of robots"
            print len(robots)
            print self.nbr_of_robots
            #Shoud probably return an error here or someting
        else:
            self.robots = robots
            self.robot = robots[0]

            set_up_services()

            self.wait_til_ready()

            #aim the robot at a ok point

            #make it move with specific speed

            #make it stop

            #make it accelerate

            #check if it reached speed

            #make it deaccelerate

            #check if it has stopped


 if __name__ == '__main__':
     try:
         rospy.init_node("Controller")
         r = PresentationProblem()

     except rospy.ROSInterruptException:
         pass

# Vad som kan brytas ut i en super Problem class
#  wait til ready metoden
#  robots variabeln
#  numberOfRobots
#  get_start_positions
