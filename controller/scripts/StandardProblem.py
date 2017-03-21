class StandardProblem(object):
    def __init__(p):
        self.p = p
        self.p.nbr_of_robots = 2

        self.r0_path = 0
        self.r1_path = 1

        a = 2.0
        self.p.start_position = [Point(-a,0.0) Point(0.0,-a)]

        path1 = [Point(a,0.0) Point(a,-a) Point(0.0,-a)]
        path2 = [Point(0.0,a) Point(-a,a) Point(-a,0.0)]

        self.paths = [path1 path2]

    def run(self, robots):
        if not robots.length==nbr_of_robots:
            print "Gave problem wrong number of robots"
            #Shoud probably return an error here or someting
        else:
            self.p.robots = robots
            self.r0 = self.p.robots[0]
            self.r1 = self.p.robots[1]

            self.p.wait_til_ready()

            self.r0.aim_at_point()

            # Set up boundary

            # Let one robot go same speed
            # Calculate what acc the other one should have.
            # Give the other that acc



    def setup_boundary
