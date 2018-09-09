from .environment import Env, Prism
import src.utils as utils

class Agent(object):

    def __init__(self, width, x, y, theta):
        self.width = width
        self.x = x
        self.y = y
        self.theta = theta
        self.boxPaths = []
        self.obstPaths = []

    def add_box_path(self, path):
        self.boxPaths.append(path)

    def add_obst_path(self, path):
        self.obstPaths.append(path)


def generate_goal(env):

    # Retrieve all box paths
    paths = env.agent.boxPaths

    # Add current obstacle paths. We do this so that successive obstacle goal locations don't intersect with obstacle
    # paths from previous function calls
    for obstaclePath in env.agent.obstPaths:
        paths.append(obstaclePath)

    # Repeatedly sample environment until non-path-intersecting sample is generated
    intersect = False
    while not intersect:
        sample = env.sample()
        # Check that sample does not intersect with any paths
        for path in paths:
            for point in range(1, len(path)):

                if Prism(env.boxes[0].width,
                             env.boxes[0].height, sample).collides_with_line(path[point - 1], path[point]):
                    intersect = True

    return sample

# def augment_path(env, path):
#
#     # Extract agent
#     agent = env.agent
#
#     # Iterate over each start position
#     for point in range(1, len(path)):
