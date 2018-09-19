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

        euclideanDistance = math.hypot(prev[0][0] - curr[0][0], prev[0][1] - curr[0][1])
        radialDistance = abs(curr[1] % math.pi - prev[1] % math.pi) * env.agent.width / 2
        out = []

        print(abs(curr[1] % (math.pi / 2) - prev[1] % (math.pi / 2)), ' == ', abs(prev[1] % (math.pi / 2) - curr[1] % (math.pi / 2)))

        stepNum = int(max(euclideanDistance, radialDistance) / 0.001)
        x = np.round(np.linspace(prev[0][0], curr[0][0], num=stepNum), decimals=3)
        y = np.round(np.linspace(prev[0][1], curr[0][1], num=stepNum), decimals=3)
        theta = np.round(np.linspace(prev[1] % math.pi, curr[1] % math.pi, num=stepNum), decimals=3)
        for i in range(0, stepNum - 1):
            out.append([(x[i], y[i]), theta[i], prev[2]])

        return out

    def move(self, env, current, currentFace, next, nextFace, box):
        print("MOVING")

        if currentFace == nextFace:
            nextNode = current
            nextNode[0] = tuple(next)
            return self.interpolate(env, current, nextNode)

        if currentFace == 't' and nextFace == 'b':
            intermediate = []
            nextNode = current.copy()
            nextNode[0] = box.get_tl()
            intermediate.extend(self.interpolate(env, current, nextNode))
            current = nextNode.copy()
            nextNode[1] = current[1] + math.pi / 2
            intermediate.extend(self.interpolate(env, current, nextNode))
            current = nextNode.copy()
            nextNode[0] = box.get_bl()
            intermediate.extend(self.interpolate(env, current, nextNode))
            current = nextNode.copy()
            nextNode[1] = current[1] + math.pi / 2
            intermediate.extend(self.interpolate(env, current, nextNode))
            current = nextNode.copy()
            nextNode[0] = tuple(next)
            intermediate.extend(self.interpolate(env, current, nextNode))
            return intermediate
        if currentFace == 'b' and nextFace == 't':
            intermediate = []
            nextNode = current.copy()
            nextNode[0] = box.get_br()
            intermediate.extend(self.interpolate(env, current, nextNode))
            current = nextNode.copy()
            nextNode[1] = current[1] + math.pi / 2
            intermediate.extend(self.interpolate(env, current, nextNode))
            current = nextNode.copy()
            nextNode[0] = box.get_tr()
            intermediate.extend(self.interpolate(env, current, nextNode))
            current = nextNode.copy()
            nextNode[1] = current[1] + math.pi / 2
            intermediate.extend(self.interpolate(env, current, nextNode))
            current = nextNode.copy()
            nextNode[0] = tuple(next)
            intermediate.extend(self.interpolate(env, current, nextNode))
            return intermediate
        if currentFace == 'l' and nextFace == 'r':
            intermediate = []
            nextNode = current.copy()
            nextNode[0] = box.get_bl()
            intermediate.extend(self.interpolate(env, current, nextNode))
            current = nextNode.copy()
            nextNode[1] = current[1] + math.pi / 2
            intermediate.extend(self.interpolate(env, current, nextNode))
            current = nextNode.copy()
            nextNode[0] = box.get_br()
            intermediate.extend(self.interpolate(env, current, nextNode))
            current = nextNode.copy()
            nextNode[1] = current[1] + math.pi / 2
            intermediate.extend(self.interpolate(env, current, nextNode))
            current = nextNode.copy()
            nextNode[0] = tuple(next)
            intermediate.extend(self.interpolate(env, current, nextNode))
            return intermediate
        if currentFace == 'r' and nextFace == 'l':
            intermediate = []
            nextNode = current.copy()
            nextNode[0] = box.get_tr()
            intermediate.extend(self.interpolate(env, current, nextNode))
            current = nextNode.copy()
            nextNode[1] = current[1] + math.pi / 2
            intermediate.extend(self.interpolate(env, current, nextNode))
            current = nextNode.copy()
            nextNode[0] = box.get_tl()
            intermediate.extend(self.interpolate(env, current, nextNode))
            current = nextNode.copy()
            nextNode[1] = current[1] + math.pi / 2
            intermediate.extend(self.interpolate(env, current, nextNode))
            current = nextNode.copy()
            nextNode[0] = tuple(next)
            intermediate.extend(self.interpolate(env, current, nextNode))
            return intermediate
        if currentFace == 't' and nextFace == 'l':
            intermediate = []
            nextNode = current.copy()
            nextNode[0] = box.get_tl()
            intermediate.extend(self.interpolate(env, current, nextNode))
            current = nextNode.copy()
            nextNode[1] = current[1] + math.pi / 2
            intermediate.extend(self.interpolate(env, current, nextNode))
            current = nextNode.copy()
            nextNode[0] = tuple(next)
            intermediate.extend(self.interpolate(env, current, nextNode))
            return intermediate
        if currentFace == 't' and nextFace == 'r':
            intermediate = []
            nextNode = current.copy()
            nextNode[0] = box.get_tr()
            intermediate.extend(self.interpolate(env, current, nextNode))
            current = nextNode.copy()
            nextNode[1] = current[1] - math.pi / 2
            intermediate.extend(self.interpolate(env, current, nextNode))
            current = nextNode.copy()
            nextNode[0] = tuple(next)
            intermediate.extend(self.interpolate(env, current, nextNode))
            return intermediate
        if currentFace == 'b' and nextFace == 'r':
            intermediate = []
            nextNode = current.copy()
            nextNode[0] = box.get_br()
            intermediate.extend(self.interpolate(env, current, nextNode))
            current = nextNode.copy()
            nextNode[1] = current[1] + math.pi / 2
            intermediate.extend(self.interpolate(env, current, nextNode))
            current = nextNode.copy()
            nextNode[0] = tuple(next)
            intermediate.extend(self.interpolate(env, current, nextNode))
            return intermediate
        if currentFace == 'b' and nextFace == 'l':
            intermediate = []
            nextNode = current.copy()
            nextNode[0] = box.get_bl()
            intermediate.extend(self.interpolate(env, current, nextNode))
            current = nextNode.copy()
            nextNode[1] = current[1] - math.pi / 2
            intermediate.extend(self.interpolate(env, current, nextNode))
            current = nextNode.copy()
            nextNode[0] = tuple(next)
            intermediate.extend(self.interpolate(env, current, nextNode))
            return intermediate
        if currentFace == 'l' and nextFace == 't':
            intermediate = []
            nextNode = current.copy()
            nextNode[0] = box.get_tl()
            intermediate.extend(self.interpolate(env, current, nextNode))
            current = nextNode.copy()
            nextNode[1] = current[1] - math.pi / 2
            intermediate.extend(self.interpolate(env, current, nextNode))
            current = nextNode.copy()
            nextNode[0] = tuple(next)
            intermediate.extend(self.interpolate(env, current, nextNode))
            return intermediate
        if currentFace == 'l' and nextFace == 'b':
            intermediate = []
            nextNode = current.copy()
            nextNode[0] = box.get_bl()
            intermediate.extend(self.interpolate(env, current, nextNode))
            current = nextNode.copy()
            nextNode[1] = current[1] + math.pi / 2
            intermediate.extend(self.interpolate(env, current, nextNode))
            current = nextNode.copy()
            nextNode[0] = tuple(next)
            intermediate.extend(self.interpolate(env, current, nextNode))
            return intermediate
        if currentFace == 'r' and nextFace == 't':
            intermediate = []
            nextNode = current.copy()
            nextNode[0] = box.get_tr()
            intermediate.extend(self.interpolate(env, current, nextNode))
            current = nextNode.copy()
            nextNode[1] = current[1] + math.pi / 2
            intermediate.extend(self.interpolate(env, current, nextNode))
            current = nextNode.copy()
            nextNode[0] = tuple(next)
            intermediate.extend(self.interpolate(env, current, nextNode))
            return intermediate
        if currentFace == 'r' and nextFace == 'b':
            intermediate = []
            nextNode = current.copy()
            nextNode[0] = box.get_br()
            intermediate.extend(self.interpolate(env, current, nextNode))
            current = nextNode.copy()
            nextNode[1] = current[1] - math.pi / 2
            intermediate.extend(self.interpolate(env, current, nextNode))
            current = nextNode.copy()
            nextNode[0] = tuple(next)
            intermediate.extend(self.interpolate(env, current, nextNode))
            return intermediate



    def add_corners(self, env, nextLine, curr):

        pos = curr[0]
        newGoal = None

        nextLine = [np.round(nextLine[0], decimals=3), np.round(nextLine[1], decimals=3)]
        # Obtain position from which to push box
        box = Prism(env.boxes[0].width, env.boxes[0].height, nextLine[0])

        if pos[0] == box.get_tl()[0] and pos[0] == box.get_bl()[0]:
            print('current pos is left')
            fr = 'l'
        elif pos[0] == box.get_tr()[0] and pos[0] == box.get_br()[0]:
            print('current pos is right')
            fr = 'r'
        elif pos[1] == box.get_tl()[1] and pos[1] == box.get_tr()[1]:
            print('current pos is top')
            fr = 't'
        else:
            print('current pos is bottom')
            fr = 'b'
        if nextLine[0][0] == nextLine[1][0]:    # y direction
            if nextLine[0][1] < nextLine[1][1]: # up
                print('next pos is top')
                to = 't'
                newGoal = (nextLine[0][0], nextLine[0][1] - box.width / 2)
            else:                               # down
                to = 'b'
                print('next pos is bottom')
                newGoal = (nextLine[0][0], nextLine[0][1] + box.width / 2)
        if nextLine[0][1] == nextLine[1][1]:    # x direction
            if nextLine[0][0] > nextLine[1][0]: # right
                to = 'r'
                print('next pos is right')
                newGoal = (nextLine[0][0] + box.width / 2, nextLine[0][1])
            else:                               # left
                to = 'l'
                print('next pos is left')
                newGoal = (nextLine[0][0] - box.width / 2, nextLine[0][1])
        if newGoal is None:



            print('curr ', curr)
            print('newGoal ', newGoal)
            print('nextLine ', nextLine)
            print('nextLine ', nextLine)
            print('currentLine ', [pos, box.centre])
            #cv2.waitKey(0)
        move = self.move(env, curr, fr, newGoal, to, box)
        return move

    def extrapolate_path(self, env, paths):
        print("EXTRAPOLATING PATHS")

        fullPath = []

        for path in paths:
            for vertex in path:
                mode = vertex[1]
                vertex = [vertex[0], utils.vector_to_object(env.get_features(), vertex[0])]
                vertex[1][1] = vertex[1][1] + math.pi / 2

                self.timeseries.append([vertex[0], vertex[1][1], mode])

        # Add corners
        for vertex in range(1, len(self.timeseries) - 2):
            prev = self.timeseries[vertex - 1]
            curr = self.timeseries[vertex]
            next = self.timeseries[vertex + 1]
            print(prev)
            print(curr)
            print(next)
            print('')
            #utils.plot_robot(env.canvas, self.width, prev)
            #utils.plot_robot(env.canvas, self.width, curr)
            fullPath.extend(self.interpolate(env, prev, curr))


            try:
                if curr[2] == 'MOVE':
                    nextLine = [self.timeseries[vertex + 1][0], self.timeseries[vertex + 2][0]]
                    extension = self.add_corners(env, nextLine, curr)
                    self.timeseries[vertex:vertex] = extension
            except:
                print("dammit")

        for config in fullPath:
            utils.plot_robot(env.canvas.copy(), self.width, config)


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
