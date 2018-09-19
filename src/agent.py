from .environment import Env, Prism
import src.utils as utils
import numpy as np
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

    def interpolate(self, env, prev, curr):

        out = []
        euclideanDistance = math.hypot(prev[0][0] - curr[0][0], prev[0][1] - curr[0][1])
        stepNum = int(euclideanDistance / 0.001)
        x = np.round(np.linspace(prev[0][0], curr[0][0], num=stepNum), decimals=3)
        y = np.round(np.linspace(prev[0][1], curr[0][1], num=stepNum), decimals=3)
        for i in range(0, stepNum - 1):
            theta = utils.vector_to_object(env.get_features(), (x[i], y[i]))
            out.append([(x[i], y[i]), theta[1] + math.pi / 2, prev[2]])

        return out



    def get_faces(self, env, nextLine, curr):

        pos = curr[0]
        nextLine = [np.round(nextLine[0], decimals=3), np.round(nextLine[1], decimals=3)]
        default = [
            False,  # 0 L
            False,  # 1 LD
            False,  # 2 D
            False,  # 3 RD
            False,  # 4 R
            False,  # 5 RU
            False,  # 6 U
            False   # 7 LU
        ]
        currentFace = default.copy()
        nextFace = default.copy()

        # Obtain position from which to push box
        box = Prism(env.boxes[0].width, env.boxes[0].height, nextLine[0])

        if pos[0] == box.get_tl()[0] and pos[0] == box.get_bl()[0]:
            print('current pos is left')
            currentFace[0] = True
        elif pos[0] == box.get_tr()[0] and pos[0] == box.get_br()[0]:
            print('current pos is right')
            currentFace[4] = True
        elif pos[1] == box.get_tl()[1] and pos[1] == box.get_tr()[1]:
            print('current pos is up')
            currentFace[6] = True
        else:
            print('current pos is down')
            currentFace[2] = True

        if nextLine[0][0] == nextLine[1][0]:    # y direction
            if nextLine[0][1] < nextLine[1][1]: # up
                print('next pos is up')
                nextFace[6] = True
                newGoal = (nextLine[0][0], nextLine[0][1] - box.width / 2)
            else:                               # down
                print('next pos is down')
                nextFace[2] = True
                newGoal = (nextLine[0][0], nextLine[0][1] + box.width / 2)
        if nextLine[0][1] == nextLine[1][1]:    # x direction
            if nextLine[0][0] > nextLine[1][0]: # right
                print('next pos is right')
                nextFace[4] = True
                newGoal = (nextLine[0][0] + box.width / 2, nextLine[0][1])
            else:                               # left
                print('next pos is left')
                nextFace[0] = True
                newGoal = (nextLine[0][0] - box.width / 2, nextLine[0][1])
        if newGoal is None:
            print("ERROR!")
            print('curr ', curr)
            print('newGoal ', newGoal)
            print('nextLine ', nextLine)
            print('nextLine ', nextLine)
            print('currentLine ', [pos, box.centre])
            #cv2.waitKey(0)
        #move = self.move(env, curr, fr, newGoal, to, box)
        return currentFace, nextFace, newGoal

    def extrapolate_path(self, env, paths):
        print("EXTRAPOLATING PATHS")
        canvas = env.canvas.copy()
        fullPath = []
        pathTracking = [] # Goddamit. Have to track paths so that intersecting obstacle doesn't confound edge adder!!

        for path in paths:
            pathTracking.append(True)   # True for start of path
            for vertex in reversed(path):
                mode = vertex[1]
                vertex = [vertex[0], utils.vector_to_object(env.get_features(), vertex[0])]
                vertex[1][1] = vertex[1][1] + math.pi / 2

                self.timeseries.append([vertex[0], vertex[1][1], mode])

                pathTracking.append(False)
            del(pathTracking[-1])


        # Add corners
        skipVerts = 0
        for vertex in range(1, len(self.timeseries) - 2):

            if vertex < skipVerts:
                pass
            else:
                prev = self.timeseries[vertex - 1]
                curr = self.timeseries[vertex]
                next = self.timeseries[vertex + 1]
                print("for vertex: ", vertex)
                print(prev)
                print(curr)
                print(next)
                print('')
                utils.plot_robot(env.canvas, self.width, prev, [255, 0, 0])
                utils.plot_robot(env.canvas, self.width, next, [0, 0, 255])
                utils.plot_robot(env.canvas, self.width, curr, [0, 255, 0])

                if vertex < 2:
                    pathCheck = True
                else:
                    pathCheck = max(pathTracking[vertex - 2: vertex + 2])

                    print("Are we in the middle of a path?? ", not max(pathTracking[vertex - 2: vertex + 2]))
                if env.box_collision_point(next[0]) and pathCheck:
                    pushBoxLocation = env.box_collision_point(next[0], getBox=True)
                    pushBox = Prism(env.boxes[0].width,
                                    env.boxes[0].height,
                                    pushBoxLocation)
                    pushBox.proximal = env.get_proximal_freedom(pushBoxLocation)
                    currentFace, nextFace, newGoal = self.get_faces(env,
                                                                    [self.timeseries[vertex + 1][0],
                                                                     self.timeseries[vertex + 2][0]],
                                                                    curr)
                    print(currentFace)
                    print(nextFace)
                    print(newGoal)
                    extension, vertexGrowth = utils.add_corners(pushBox,
                                                                curr,
                                                                currentFace,
                                                                nextFace,
                                                                newGoal)

                    if extension is not None:
                        print("adding corners. vertex: ", vertex, ' len(addition): ', vertexGrowth)
                        print("extension: ", extension)
                        print("before adding: (len) ", len(self.timeseries))
                        for row in self.timeseries[vertex - 1 - vertexGrowth:vertex + vertexGrowth + 1]:
                            print(self.timeseries.index(row), row)
                        #del(self.timeseries[vertex])
                        self.timeseries[vertex + 1:vertex + 1] = extension
                        pathTracking[vertex + 1:vertex + 1] = [True for x in range(0, vertexGrowth)]
                        vertex += vertexGrowth
                        #del (self.timeseries[vertex:vertexGrowth])
                        print("added corners, updated vertex position to: ", vertex)
                        print("aftere adding: (len) ", len(self.timeseries))
                        for row in self.timeseries[vertex - 4 - vertexGrowth:vertex + vertexGrowth + 1]:
                            print(self.timeseries.index(row), row)
                        # next = self.timeseries[vertex + 1]
                        # utils.plot_robot(env.canvas, self.width, next, [0, 0, 255])
                        # cv2.waitKey(0)

                #cv2.waitKey(0)
                #fullPath.extend(self.interpolate(env, prev, curr))
                skipVerts = vertex
        print("done adding corners")
        env.canvas = canvas
        for vertex in self.timeseries:
            cv2.waitKey(0)
            utils.plot_robot(env.canvas, self.width, vertex, [0, 0, 0])

        for config in fullPath:
            utils.plot_robot(env.canvas.copy(), self.width, config, [0, 0, 0])


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
