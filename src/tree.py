from itertools import chain
from builtins import map
import math
import random
import cv2
import src.vistools as vis
import src.utils as utils
import src.agent as agent
from .environment import Prism
import math

class Node(object):

    def __init__(self, start, goal, cost):
        self.distance = math.hypot(goal[0] - start[0], goal[1] - start[1])
        self.start = start  # Node position
        self.goal = goal    # Goal position
        self.id = id(self)  # Unique identifier for class instance
        self.children = []
        self.parent = None  # Used to trace lineage from goal back to start
        self.done = False   # Only true for goal node

    def __iter__(self):
        for v in chain(*map(iter, self.children)):
            yield v
        yield self

    def add_child(self, obj):
        self.children.append(obj)


def expand_tree(node):

    pathReversed = []
    pathReversed.append([node.start, True])
    while node.parent is not None:
        pathReversed.append([node.parent.start, node.theta])
        node = node.parent

    return pathReversed


def generate_node(env, parent, sample, stepSize, clearance, startIgnore=None, endIgnore=None, fin=False, robot=False):

    deltaX = sample[0] - parent.start[0]
    deltaY = sample[1] - parent.start[1]

    if abs(deltaX) > abs(deltaY):   # Generate new sample on X or Y axis of parent node. Randomise new edge length.

        if fin:
            newX = parent.start[0] + deltaX
            newY = parent.start[1]
        else:
            newX = parent.start[0] + stepSize * utils.sign(deltaX) * random.random()
            newY = parent.start[1]
    else:
        if fin:
            newY = parent.start[1] + deltaY
            newX = parent.start[0]
        else:
            newY = parent.start[1] + stepSize * utils.sign(deltaY) * random.random()
            newX = parent.start[0]

    candidateBox = Prism(clearance, clearance, (newX, newY))
    if not env.static_collision(candidateBox):
        # if fin:
        #     print(sample, 'point ', (newX, newY), ' did not collide with static, with clearance ', clearance)
        #     cv2.waitKey(0)
        # if clearance == 0.001:
        #     print('ok')
        if not env.box_collision(candidateBox, startIgnore, endIgnore, robot):
            # if fin:
            #     print(sample, 'point ', (newX, newY), ' did not collide with box, with clearance ', clearance)
            #     cv2.waitKey(0)
            if not env.goal_collision(candidateBox, parent.goal):
                # if fin:
                #
                #     print(sample, '\t\tpoint ', (newX, newY), ' did not collide with goal, with clearance ', clearance)
                #     cv2.waitKey(0)



                update = Node((newX, newY), parent.goal, math.hypot(newX - parent.goal[0], newY - parent.goal[1]))
                # update = Node((parent.start[0], newY), parent.goal, parent.cost + newY - parent.start[0])
                # update = Node((newX, parent.start[1]), parent.goal, parent.cost + newX - parent.start[0])
                update.parent = parent
                return update
    return None


def check_node_against_goals(env, pos, currentGoal):

    for box in env.boxes:

        # only check other box-goals
        if box.goal[0] != currentGoal[0] and box.goal[1] != currentGoal[1]:
            if Prism(box.width, box.height, pos).collides_with_box(Prism(box.width, box.height, box.goal)):
                return False

    for box in env.obstacles:

        # Don't do anything for obstacles that don't have goals
        if box.obstacleGoal:

            # only check other box-goals
            if box.obstacleGoal[0] != currentGoal[0] and box.obstacleGoal[1] != currentGoal[1]:
                if Prism(box.width, box.height, pos).collides_with_box(
                        Prism(box.width * 2.0, box.height * 2.0, box.obstacleGoal)):
                    return False

    return True


def generate_path(env, startNode, endNode, stepSize, clearance, plot=False, robot=False):
    #
    # print("generating path for robot: ", robot, len(env.agent.finalPath))
    cleanCanvas = env.canvas.copy()
    env.update_canvas()

    if robot:
        env.numRobotPaths += 1

        # Ensure that robot path starts at box edge
        if env.numRobotPaths == 1:
            endNode = (endNode[0] - env.boxes[0].width/2, endNode[1])
        else:
            endNode = (endNode[0] - env.boxes[0].width / 2, endNode[1])
            startNode = (startNode[0] - env.boxes[0].width / 2, startNode[1])

    # Store euclidean distance heuristic in root node
    newTree = Node(startNode, endNode, 0)
    newTree.root = startNode
    env.add_tree(newTree)
    finalPoint = False
    robotFinalPoint = True

    while True:

        if finalPoint:
            pass
        else:
            # Generate valid sample while candidate point not in collision with world
            samplePoint = env.sample()
        if plot:
            canvas = env.show(id(newTree))
            cv2.imshow('environment', vis.plot_sample(canvas.copy(), samplePoint))
            cv2.waitKey(1)

        # Select closest node to sample point by iterating through nodes
        closestNode = 1
        for node in iter(newTree):
            sampleDistance = math.hypot(samplePoint[0] - node.start[0], samplePoint[1] - node.start[1])
            if sampleDistance <= closestNode:
                closestNode = sampleDistance

        # And then update node with sample point at position
        for node in iter(newTree):
            sampleDistance = math.hypot(samplePoint[0] - node.start[0], samplePoint[1] - node.start[1])
            if sampleDistance == closestNode:   # add child
                newNode = generate_node(env, node, samplePoint, stepSize, clearance, startNode, endNode, finalPoint, robot)

                if newNode is not None:

                    if robot and env.box_collision_point(newNode.start):
                        finalPoint = False
                        break
                    else:

                        newNode.theta = robot
                        node.add_child(newNode)
                        newNode.root = node.root

                if node.distance <= stepSize:
                    finalPoint = True
                    samplePoint = node.goal

            # Check for win condition
            if node.distance == 0:
                path = expand_tree(node)
                if plot:
                    env.update_canvas()
                    cv2.imshow('environment', env.canvas)
                    #cv2.waitKey(0)
                env.canvas = cleanCanvas
                return path
