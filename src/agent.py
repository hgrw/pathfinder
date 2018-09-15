from .environment import Env, Prism
import src.utils as utils
import src.vistools as vis
import cv2
import math
import random

class Agent(object):

    def __init__(self, width, x, y, theta):
        self.width = width
        self.x = x
        self.y = y
        self.theta = theta
        self.finalPath = []
        self.timeseries = []

    # def add_box_path(self, path):
    #     self.boxPaths.append(path)

    # def add_obst_path(self, path):
    #     self.obstPaths.append(path)
    def update_paths(self, env):

        canvas = env.canvas.copy()
        newPaths = []
        deleteMode = 1

        # Get each path segment
        for pathPos in range(1, len(self.finalPath)):
            dummy = []

            # Reverse path segment
            for vert in reversed(self.finalPath[pathPos - 1]):
                dummy.append(vert)
                path = dummy

            # Get each vertex in reversed path segment
            for vertex in range(1, len(path)):

                line = [path[vertex - 1][0], path[vertex][0]]

                if pathPos == 1:    # Ignore intersection at starting vertex
                    intersection = env.box_intersection(line, [Prism(env.boxes[0].width, env.boxes[0].height, path[-1][0])])
                else:
                    intersection = env.box_intersection(line, [Prism(env.boxes[0].width, env.boxes[0].height, path[-1][0]),
                                                               Prism(env.boxes[0].width, env.boxes[0].height, path[0][0])])

                # If we're a robot path,
                # stop adding vertices and instead complete path using subroutine for angling to next position
                if intersection is not None and path[vertex][1]:
                    # print("path ", path)
                    # cv2.imshow('environment', cv2.circle(canvas, utils.scale(intersection), 3, [0, 0, 0], -1))
                    # cv2.waitKey(0)

                    if deleteMode % 2:  # Delete after
                        path = path[:vertex - 1]
                        path.append([intersection, 'MOVE'])

                    else:               # Delete before
                        path = [[intersection, 'MOVE']] + path[vertex:]

                    # print('path ', path)
                    # cv2.waitKey(0)
                    deleteMode += 1

                    break
            newPaths.append(path)

        print(self.finalPath)
        print(newPaths)
        self.finalPath = newPaths


    def extrapolate_path(self, env, path):

        for vertex in path:
            vertex = [vertex[0], utils.vector_to_object(env.get_features(), vertex[0])]
            vertex[1][1] = vertex[1][1] + math.pi / 2
            self.timeseries.append(vertex)

            pt1, pt2 = utils.robot_line(self.width, vertex)
            cv2.imshow('environment', cv2.circle(cv2.line(env.canvas,
                                                          (int(pt1[0] * 1000), int(pt1[1] * 1000)),
                                                          (int(pt2[0] * 1000), int(pt2[1] * 1000)), [0, 0, 0], 2),
                                                 (int(vertex[0][0] * 1000), int(vertex[0][1] * 1000)), 5, [0, 0, 0],
                                                 -1))
            cv2.waitKey(0)

        #vis.animate_path(env.agent, env.canvas.copy(), path)
        # cv2.imshow('environment', env.canvas)
        # cv2.waitKey(0)



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
