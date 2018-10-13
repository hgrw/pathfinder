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

    def interpolate(self, env, prev, curr, force=None):

        if force is not None:
            print("problem. prev: ", prev)
            print("curr: ", curr)

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

        if env.box_collision_point(pos):
            centre =  env.box_collision_point(pos, getBox=True)
        elif env.box_collision_point(nextLine[0]):
            centre = env.box_collision_point(nextLine[0], getBox=True)
        else:
            print('failed to get box')
            print('pos ', pos)
            print('nextLine ', nextLine)
            exit(0)

        print('got box at: ', centre)

        # Obtain position from which to push box
        box = Prism(env.boxes[0].width, env.boxes[0].height, centre)

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
        print('analysed faces')
        print('pos ', pos)
        print('nextLine ', nextLine)
        return currentFace, nextFace, newGoal

    def extrapolate_path(self, env, paths):
        print("EXTRAPOLATING PATHS")
        canvas = env.canvas.copy()
        boxWidth = env.boxes[0].width / 2
        pathTracking = [] # Goddamit. Have to track paths so that intersecting obstacle doesn't confound corner adder!!

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

            if vertex <= skipVerts:
                pass
            else:
                prev = self.timeseries[vertex - 1]
                curr = self.timeseries[vertex]
                next = self.timeseries[vertex + 1]
                print("for vertex: ", vertex)
                try:
                    print("at location, relative to paths: ", pathTracking[vertex-1], pathTracking[vertex],
                          pathTracking[vertex + 1])
                except:
                    pass
                print(prev)
                print(curr)
                print(next)
                print('')
                # utils.plot_robot(env.canvas, self.width, prev, [255, 0, 0])
                # utils.plot_robot(env.canvas, self.width, next, [0, 0, 255])
                # utils.plot_robot(env.canvas, self.width, curr, [0, 255, 0])

                # If the next point (i.e. curr in the next iteration) is in the box, we add
                # extra corners!!
                if vertex < 2:
                    pathCheck = True
                else:
                    # If the next point is the start of a new path, we want to be adding corners!
                    pathCheck = pathTracking[vertex + 1]

                    print("Are we in the middle of a path?? ", not max(pathTracking[vertex - 2: vertex + 2]))
                if env.box_collision_point(next[0]) and pathCheck:
                    print('we are adding corners for line segment: ', curr[0], next[0])
                    #cv2.waitKey(0)
                    pushBoxLocation = env.box_collision_point(next[0], getBox=True)
                    pushBox = Prism(env.boxes[0].width,
                                    env.boxes[0].height,
                                    pushBoxLocation)
                    pushBox.proximal = env.get_proximal_freedom(pushBoxLocation)
                    currentFace, nextFace, newGoal = self.get_faces(env,
                                                                    [self.timeseries[vertex + 1][0],
                                                                     self.timeseries[vertex + 2][0]],
                                                                    curr)
                    extension, vertexGrowth = utils.add_corners(pushBox,
                                                                curr,
                                                                currentFace,
                                                                nextFace,
                                                                newGoal)

                    if extension is not None:
                        # print("current face\t", currentFace)
                        # print("next face \t", nextFace)
                        # print("adding corners. vertex: ", vertex, ' len(addition): ', vertexGrowth)
                        # print("before adding: (len) ", len(self.timeseries))
                        # print('bef all ', [[self.timeseries[x][0], x] for x in
                        #                range(max(0, vertex - 1), min(len(self.timeseries), vertex + 8))])
                        # #del(self.timeseries[vertex])
                        # print('ext ', extension)
                        self.timeseries[vertex + 1:vertex + 1] = extension

                        print('aft all ', [[self.timeseries[x][0], x] for x in
                                       range(max(0, vertex - 1),
                                             min(len(self.timeseries), vertex + 5 + vertexGrowth))])

                        #cv2.waitKey(0)
                        pathTracking[vertex + 1:vertex + 1] = [True for x in range(0, vertexGrowth)]
                        vertex += vertexGrowth
                        #del (self.timeseries[vertex:vertexGrowth])
                        # print("added corners, updated vertex position to: ", vertex)
                        # print("aftere adding: (len) ", len(self.timeseries))
                        # for row in self.timeseries[vertex - 4 - vertexGrowth:vertex + vertexGrowth + 1]:
                        #     print(self.timeseries.index(row), row)
                        # next = self.timeseries[vertex + 1]
                        # utils.plot_robot(env.canvas, self.width, next, [0, 0, 255])
                        #cv2.waitKey(0)

                #cv2.waitKey(0)
                #fullPath.extend(self.interpolate(env, prev, curr))
                skipVerts = vertex
        print("done adding corners")
        dots = np.ones((1000, 1000, 3), dtype=np.uint8) * 255
        for vertex in self.timeseries:
            dots = cv2.circle(dots, utils.scale(vertex[0]), 3, [0, 0, 0], -1)

        env.canvas = canvas
        fullPath = []


        # Remove paths from environment
        for box in env.boxes:
            box.path = None
        for box in env.obstacles:
            box.path = None

        for vertex in range(1, len(self.timeseries) - 1):
            prev = self.timeseries[vertex - 1]
            curr = self.timeseries[vertex]
            next = self.timeseries[vertex + 1]
            dist = self.timeseries[vertex + 2]
            agentNext = next.copy()
            agentDist = dist.copy()
            # print("prev: ", prev)
            # print("curr: ", curr)
            # print("next: ", next)

            # Get current and next orientation for each line segment
            prevOrientation, currOrientation = utils.get_orientation(prev[0], curr[0], next[0])
            nextOrientation, distOrientation = utils.get_orientation(next[0], dist[0], self.timeseries[vertex + 3][0])

            fullPath.extend(self.interpolate(env, prev, curr))


            if not next[2]:     # Push mode engage
                # for config in fullPath:
                #     utils.plot_robot(env.canvas.copy(), self.width, config, [0, 0, 0])
                print("PUSH MODE ENGAGED ON SEGMENT: ")
                print("prev: ", prev)
                print('curr: ', self.timeseries[vertex][0], currOrientation)
                print('next: ', self.timeseries[vertex + 1][0], nextOrientation)
                print('dist: ', self.timeseries[vertex + 2][0], distOrientation)


                # These two lines need to be smarter. Instead of just interpolating to the next point, (which for agent is
                # just a half box width away), we need to interpolate to the next corner in the box path!

                # How far away is the next box vert?

                if currOrientation == nextOrientation:
                    boxVerts = self.interpolate(env, next, dist)
                    agentNext[0] = utils.update_point(next[0], nextOrientation, boxWidth)
                    agentDist[0] = utils.update_point(dist[0], nextOrientation, boxWidth)
                    agentVerts = self.interpolate(env, agentNext, agentDist)
                else:
                    print("Turning the corner on this fucking thing")
                    cv2.waitKey(0)
                    # get box
                    boxPos = env.box_collision_point(next[0], getBox=True)
                    # Find which box we're pushing
                    if not boxPos:
                        print('boxpos: ', boxPos)
                        print('Fell off box, looking at: ', curr[0])
                        print('alternative prev, next, dist', prev[0], next[0], dist[0])
                        cv2.waitKey(0)
                        exit(0)
                    else:
                        boxIndex, boxType = env.get_object_index(boxPos)
                        box = env.get_box(boxIndex, boxType)

                    # Get pushing position on box
                    if currOrientation[4]:  # On left face
                        pos = box.get_l()
                        currOrientation[4] = False
                        currOrientation[0] = True
                    elif currOrientation[0]:    # On right face
                        currOrientation[0] = False
                        currOrientation[4] = True
                        pos = box.get_r()
                    elif currOrientation[2]:    # On top face
                        currOrientation[2] = False
                        currOrientation[6] = True
                        pos = box.get_t()
                    else:
                        currOrientation[6] = False
                        currOrientation[2] = True
                        pos = box.get_b()
                    dummy = curr.copy()
                    dummy[0] = pos
                    print('got pos: ', pos)

                    # Get new goal
                    _, _, newGoal = self.get_faces(env, [next[0], dist[0]], dummy)
                    print("got goal! ", newGoal)

                    # Add corners
                    extension, vertexGrowth = utils.add_corners(box, dummy, currOrientation, nextOrientation, newGoal)

                    # Extend fullpath with corners
                    if extension is not None:
                        print('boxpos: ', boxPos)
                        print("current face\t", currOrientation)
                        print("next face \t", nextOrientation)
                        print("before adding: (len) ", len(self.timeseries))
                        print('bef all ', [[self.timeseries[x][0], x] for x in
                                       range(max(0, vertex - 1), min(len(self.timeseries), vertex + 8))])
                        #del(self.timeseries[vertex])
                        print('ext ', [extension[x][0] for x in range(0, len(extension))])
                        for vert in range(0, len(extension) - 1):
                            fullPath.extend(self.interpolate(env, extension[vert], extension[vert + 1]))
                    print("added corners: ", extension)

                    # Update agent and box trajectory
                    boxVerts = self.interpolate(env, next, dist)
                    agentNext[0] = utils.update_point(next[0], distOrientation, boxWidth)
                    agentDist[0] = utils.update_point(dist[0], distOrientation, boxWidth)
                    agentVerts = self.interpolate(env, agentNext, agentDist)
                    cv2.waitKey(0)

                #print('all ', [self.timeseries[x][0] for x in range(max(0, vertex - 5), min(len(self.timeseries), vertex + 8))])
                print("For Vertex: ", vertex)
                print("AGENT GOING FROM: ", agentVerts[0][0], "  TO: ", agentVerts[-1][0], ' Len: ', len(agentVerts))
                print("__BOX GOING FROM: ", boxVerts[0][0], "  TO: ", boxVerts[-1][0], ' Len: ', len(boxVerts))
                print("_BOTH GOING FROM: ", min(len(agentVerts), len(boxVerts)))

                cv2.imshow('environment', env.canvas & dots)
                cv2.waitKey(0)
                for config in range(1, len(boxVerts) - 2):

                    # Separate primitives
                    prevConfig = agentVerts[config - 1]
                    currConfig = agentVerts[config]
                    nextConfig = agentVerts[config + 1]

                    #cv2.waitKey(0)
                    boxPos = env.box_collision_point(boxVerts[config + 1][0], getBox=True)

                    # Find which box we're pushing
                    if not boxPos:
                        print('boxpos: ', boxPos)
                        print('Fell off box, looking at: ', boxVerts[config + 1][0])
                        print('alternative boxVerts[config + 1][0]: ', boxVerts[config + 1][0])
                        print("BOX POSITIONS: ")
                        for box in env.obstacles:
                            print(box.centre)
                        for box in env.boxes:
                            print(box.centre)
                        #print('alternative boxVerts[config + 2][0]: ', boxVerts[config + 2][0])
                        cv2.waitKey(0)
                        exit(0)
                    else:
                        boxIndex, boxType = env.get_object_index(boxPos)

                    # print("from: ", prevConfig[0], " to: ", currConfig[0], ' next: ', nextConfig[0])
                    if boxType == 'obs':
                        updateBox = Prism(env.boxes[0].width, env.boxes[1].height, boxVerts[config][0])
                        updateBox.path = None
                        updateBox.obstacleGoal = env.obstacles[boxIndex].obstacleGoal
                        env.obstacles[boxIndex] = updateBox

                    env.update_canvas()
                    env.canvas = env.canvas & dots

                    #if not env.box_collision_point(prevConfig[0]):
                    fullPath.extend(prevConfig)
                    #print('added: ', prevConfig)
                    #cv2.imshow('environment', env.canvas)
                    utils.plot_robot(env.canvas, self.width, agentVerts[config], [0, 0, 0])
                    #cv2.waitKey(0)

                if not next[2]:  # Tack on the last few points when agent is pushing box
                    fullPath.extend(prevConfig)
                    #print('added: ', prevConfig)
                    fullPath.extend(currConfig)
                    #print('added: ', currConfig)
                    fullPath.extend(nextConfig)
                    #print('added: ', nextConfig)

                # Add last two portions to agent movement

                # NOTE, ^^ these two steps need to be taken into account for the box somehow
                # print("NEED TO UPDATE NEW POSITIONS: ")
                # print("config: ", config)
                # print("agentVerts[config]: ", nextConfig)
                # print("agentVerts[-1]: ", agentVerts[-1])
                # print("boxVerts[config]:", boxVerts[config])
                # print("boxVerts[-1]:", boxVerts[-1])
                # cv2.waitKey(0)



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
