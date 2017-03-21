class PresetationProblem(object):
    def __init__(p):
        self.p = p
        self.p.nbr_of_robots = 1
        self.p.start_position = [Point()]

    def run(self, robots):
        if not robots.length==nbr_of_robots:
            print "Gave problem wrong number of robots"
            #Shoud probably return an error here or someting
        else:
            self.p.robots = robots
            self.robot = robots[0]

            self.p.wait_til_ready()

            #aim the robot at a ok point

            #make it move with specific speed

            #make it stop

            #make it accelerate

            #check if it reached speed

            #make it deaccelerate

            #check if it has stopped




# Vad som kan brytas ut i en super Problem class
#  wait til ready metoden
#  robots variabeln
#  numberOfRobots
#  get_start_positions

# Men men    inheritance är ju ändå inte så vidare snyggt så vet inte om jag vill ha det
