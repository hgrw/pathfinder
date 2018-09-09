import numpy as np
import cv2
import random
import src.vistools as vis
import src.utils as utils

class Env(object):

    statics = []

    def __init__(self):
        self.boxes = []
        self.obstacles = []
        self.agent = None
        self.trees = {}
        self.primitive = 0.001

    def add_box(self, box):
        self.boxes.append(box)

    def add_obst(self, obstacle):
        self.obstacles.append(obstacle)

    def add_static(self, static):
        self.statics.append(static)

    def add_agent(self, agent):
        self.agent = agent

    def add_tree(self, tree):
        self.trees[id(tree)] = tree

    def collides_with_point(self, point):

        for static in self.statics:
            if static.collides_with(point):
                return True

    def collides_with_box(self, box):

        for static in self.statics:
            if (static.xMin < box.xMin < static.xMax or static.xMin < box.xMax < static.xMax) and \
                    (static.yMin < box.yMin < static.yMax or static.yMin < box.yMax < static.yMax):
                return True

    def sample(self):

        # set the clearance so that you can wedge your thumb between a box centred at sample point and world edge [0, 1]
        clearance = self.boxes[0].width/1.9

        while True:
            # Generate candidate with clearance
            candidate = (random.random() * (1 - 2 * clearance) + clearance,
                         random.random() * (1 - 2 * clearance) + clearance)

            return candidate

    def show(self, dimms, treeID=False):
        canvas = np.ones(dimms, dtype=np.uint8) * 255

        for box in range(0, len(self.boxes)):
            canvas = vis.plot_box(canvas, dimms, [[255, 0, 0], [0, 0, 255]],
                                  self.boxes[box].get_tl(dimms[0]),
                                  self.boxes[box].get_br(dimms[0]),
                                  goal=self.boxes[box].get_goal(dimms[0]))

        for box in range(0, len(self.obstacles)):
            canvas = vis.plot_box(canvas, dimms, [[255, 0, 0], [0, 0, 255]],
                                  self.obstacles[box].get_tl(dimms[0]),
                                  self.obstacles[box].get_br(dimms[0]),
                                  obstacle=True)

        for box in range(0, len(self.statics)):
            canvas = vis.plot_box(canvas, dimms, [[0, 0, 0], [0, 0, 0]],
                                  self.statics[box].get_tl(dimms[0]),
                                  self.statics[box].get_br(dimms[0]))

        if treeID is not False:

            for node in iter(self.trees[treeID]):

                # Plot node position
                canvas = vis.plot_sample(canvas, node.start, (1000, 1000, 3))

                # plot line to children
                for child in node.children:
                    canvas = cv2.line(canvas,
                                      (int(node.start[0] * 1000), int(node.start[1] * 1000)),
                                      (int(child.start[0] * 1000), int(child.start[1] * 1000)), [0, 0, 0], 2)
        return canvas


class Prism(object):

    def __init__(self, width, height, centre):
        self.width = width
        self.height = height
        self.centre = centre
        self.xMin = centre[0] - width / 2
        self.xMax = centre[0] + width / 2
        self.yMin = centre[1] - width / 2
        self.yMax = centre[1] + width / 2
        self.obstacleGoal = None

    def add_goal(self, goal):
        self.obstacleGoal = goal

    def get_tl(self, scale):
        return (self.centre[0] - self.width/2) * scale, (self.centre[1] - self.width/2) * scale

    def get_tr(self, scale):
        return (self.centre[0] + self.width/2) * scale, (self.centre[1] - self.width/2) * scale

    def get_bl(self, scale):
        return (self.centre[0] - self.width/2) * scale, (self.centre[1] + self.height/2) * scale

    def get_br(self, scale):
        return (self.centre[0] + self.width/2) * scale, (self.centre[1] + self.height/2) * scale

    def collides_with_box(self, box):
        if (self.xMin < box.xMin < self.xMax or self.xMin < box.xMax < self.xMax) and \
                (self.yMin < box.yMin < self.yMax or self.yMin < box.yMax < self.yMax):
            return True
        else:
            return False

    def collides_with_point(self, point):
        return point[0] > self.xMin and point[0] < self.xMax and point[1] > self.yMin and point[1] < self.yMax

    def collides_with_line(self, p1, p2):

        if p1[0] == p2[0]:  # Vertical line
            p1, p2 = utils.order(p1, p2)  # ensure p1 < p2
            return (p1[1] < self.yMin < p2[1] or p1[1] < self.yMax < p2[1]) and self.xMin < p1[0] < self.xMax
        else:               # Horizontal line
            p1, p2 = utils.order(p1, p2)  # ensure p1 < p2
            return (p1[0] < self.xMin < p2[0] or p1[0] < self.xMax < p2[0]) and self.yMin < p1[1] < self.yMax


class Moving(Prism):

    def __init__(self, side, centre, goal):
        super().__init__(side, side, centre)
        self.sideLength = side
        self.centre = centre
        self.goal = goal

    def get_goal(self, scale):
        return (self.goal[0] - self.sideLength/2) * scale, (self.goal[1] - self.sideLength/2) * scale