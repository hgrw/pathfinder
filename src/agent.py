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


def box_intersect_path(paths, prism):

    for path in paths:
        for point in range(1, len(path)):

            if prism.collides_with_line(path[point - 1], path[point]):
                return True

    return False


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
        intersect = box_intersect_path(paths, Prism(env.boxes[0].width, env.boxes[0].height, sample))

    return sample


def get_rotation_locus(env, current, next, pt1, pt2, pt3):
    if current == 0:  # left
        if env.collides_with_box(Prism())


def update_path(paths, env):



    # at corner, identify current face and next face


    # identify rotation locus (either at current face or at next face)


    # generate transition matrix and append to updated path


    for path in paths:

        updatedPath = []
        for point in range(1, len(path - 1)):
            currentFace = utils.get_face(path[point - 1], path[point])
            nextFace = utils.get_face(path[point], path[point + 1])

            rotationLocus = get_rotation_locus(env, currentFace, nextFace,
                                                     path[point - 1], path[point], path[point + 1])



    # Extract agent
    agent = env.agent

    # Iterate over each start position
    for point in range(1, len(path)):
