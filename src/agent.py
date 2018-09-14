from .environment import Env, Prism
import src.utils as utils
import random

class Agent(object):

    def __init__(self, width, x, y, theta):
        self.width = width
        self.x = x
        self.y = y
        self.theta = theta
        self.finalPath = []

    # def add_box_path(self, path):
    #     self.boxPaths.append(path)

    # def add_obst_path(self, path):
    #     self.obstPaths.append(path)


def box_intersect_path(paths, prism):
    for path in paths:
        for point in range(1, len(path)):

            # Hack to make sure that lines close to prism are flagged as intersections
            hack = Prism(prism.width * 2.0, prism.height * 2.0, prism.centre)
            if hack.collides_with_line(path[point - 1], path[point]):
                return True

    return False


def box_intersect_goals(env, sampleBox):

    boxLoci = []
    for box in env.boxes:
        boxLoci.append(box)

    for box in env.obstacles:
        boxLoci.append(box)

    for box in boxLoci:

        if box.collides_with_box(sampleBox):
            return False
        else:
            return True


def generate_goal(env):

    paths = []

    # Add current obstacle paths. We do this so that successive obstacle goal locations don't intersect with obstacle
    # paths from previous function calls
    for box in env.boxes:
        if box.path:
            paths.append(box.path)

    for box in env.obstacles:
        if box.path:
            paths.append(box.path)

    # Generate sample point that does not intersect with paths
    intersect = True
    while intersect is True:

        # Get sample and goal candidate,
        sample = env.sample()
        goalCandidate = Prism(env.boxes[0].width * 2.0, env.boxes[0].height * 2.0, sample)
        smallCandidate = Prism(env.boxes[0].width, env.boxes[0].height, sample)
        # If candidate doesn't intersect statics
        if not env.static_collision(goalCandidate) and not env.box_collision(smallCandidate, None, None) and not env.goal_collision(smallCandidate, None):
            intersect = False
            for path in paths:
                for point in range(1, len(path)):
                    if goalCandidate.collides_with_line(path[point - 1][0], path[point][0]):
                        intersect = True
    return sample



def get_rotation_locus(env, current, next, pt1, pt2, pt3):
    if current == 0:  # left
        pass
        #if env.collides_with_box(Prism())


def update_path(env, paths):


    exit(0)
    # at corner, identify current face and next face


    # identify rotation locus (either at current face or at next face)


    # generate transition matrix and append to updated path


    for path in paths:

        updatedPath = []
        for point in range(1, len(path - 1)):
            currentFace = utils.get_face(path[point - 1], path[point])
            nextFace = utils.get_face(path[point], path[point + 1])
    #
    #         rotationLocus = get_rotation_locus(env, currentFace, nextFace,
    #                                                  path[point - 1], path[point], path[point + 1])
    #
    #
    #
    # # Extract agent
    # agent = env.agent
    #
    # # Iterate over each start position
    # for point in range(1, len(path)):
