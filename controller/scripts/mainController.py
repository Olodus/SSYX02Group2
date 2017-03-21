
# Only supports 1 or 2 robots for now
def simulator_setup(nbr_of_robots, start_positions):
    # Create pose handlers
     # subscribe them to pose
     # give correct offset if several
    # Create robot handlers
     # make them subscribe to correct PoseHandler
     # make robots move towards their startPoint

    robots = [nbr_of_robots]
    for i in range(0,nbr_of_robots-1):
        offsetX = 0.0
        offsetY = 0.0
        if i = 1:
            offsetY = 1.0

        h = PoseHandler(i,offsetX,offsetY)
        robots[i] = RobotHandler(i)
        robots[i].go_to_point(start_positions[i])
        robots[i].run()

    return robots

def real_world_setup(nbr_of_robots):
    # Create UWBHandlers
     # subscribe them to pose
    # Create RobotHandlers
    # Make them subscribe to UWB
    # Make them move towards start_positions
    return robots

if __name__ == '__main__':
    try:
        rospy.init_node('controller', anonymous=True)

        super_problem = Problem()
        problem = PresetationProblem(super_problem)
        nr = super_problem.numberOfRobots()
        start = super_problem.get_start_positions()

        robots = simulator_setup(nr, start)
        #robots = real_world_setup(nr)

        problem.run(robots)
    except rospy.ROSInterruptException:
        pass
