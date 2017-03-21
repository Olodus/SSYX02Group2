# The super class of a problem
# is handled as a decoration in the defined problems
# this is definatly not a good way to handle inheritance...
# probably should fix this.

class Problem(object):
    def __init__():
        self.nbr_of_robots = 0
        self.start_position = []
        self.robots = []

    def numberOfRobots(self):
        return nbr_of_robots

    def get_start_positions(self):
        return start_position

    def wait_til_ready(self):
        # loop through and see if all robots are ready
