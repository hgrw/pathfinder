from itertools import chain
from builtins import map
import math
import random
import cv2

class Node(object):
    def __init__(self, distance, x, y):
        self.distance = distance
        self.x = x
        self.y = y
        self.children = []

    def __iter__(self):
        "implement the iterator protocol"
        for v in chain(*map(iter, self.children)):
            yield v
        yield self

    def add_child(self, obj):
        self.children.append(obj)


def sign(x):
    return (1 - (x <= 0))*2 - 1


def generate_node(parent, sample, stepSize, boxEnd, plot=False):

    deltaX = parent[0] - sample[0]
    deltaY = parent[1] - sample[1]

    if abs(deltaX) < abs(deltaY):   # Generate new sample on x axis
        newX = parent[0] + stepSize * random.random() * sign(deltaX)
        update = Node(math.hypot(newX - boxEnd[0], parent[1] - boxEnd[1]), newX, parent[1])
    else:
        newY = parent[1] + stepSize * random.random() * sign(deltaY)
        update = Node(math.hypot(parent[0] - boxEnd[0], newY - boxEnd[1]), parent[0], newY)

    if plot.any():
        # cv2.imshow('environment', plot)
        # cv2.waitKey(0)
        point = (int(update.x*1000), int(update.y*1000))
        #print(point)
        cv2.imshow('environment', cv2.circle(cv2.circle(plot, point, 5, [120, 0, 100], -1),
                                             (int(sample[0] * 1000), int(sample[1] * 1000)),
                                             5, [120, 185, 100], -1))
        cv2.waitKey(0)
    return update


def generate_path(env, boxStart, boxEnd, stepSize):

    while True:

        # Store euclidean distance heuristic in root node
        root = Node(math.hypot(boxEnd[0] - boxStart[0], boxEnd[1] - boxStart[1]), boxStart[0], boxStart[1])

        # Generate valid sample while candidate point not in collision with world
        samplePoint = env.sample()    # just get any point for now

        # Select closest node to sample point by iterating through nodes
        closestNode = 1
        for node in iter(root):
            sampleDistance = math.hypot(samplePoint[0] - node.x, samplePoint[1] - node.y)
            if sampleDistance <= closestNode:
                closestNode = sampleDistance

        # And then update node with sample point at position
        for node in iter(root):
            if math.hypot(samplePoint[0] - node.x, samplePoint[1] - node.y) == closestNode: # add child
                node.add_child(generate_node((node.x, node.y),
                                             samplePoint,
                                             stepSize,
                                             boxEnd,
                                             env.show((1000, 1000, 3))))

            # Check for win condition
            if node.distance <= stepSize:
                return root
