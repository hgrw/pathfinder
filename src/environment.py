import numpy as np
import cv2
import math
import random
import src.vistools as vis
import src.utils as utils


class Env(object):

    statics = []
    colours = []

    def __init__(self):
        self.boxes = []
        self.obstacles = []
        self.agent = None
        self.currentGoal = 0
        self.trees = {}
        self.treeIDs = []
        self.primitive = 0.001
        self.canvas = np.ones((1000, 1000, 3), dtype=np.uint8) * 255
        self.colours.append([0, 0, 255])
        self.colours.append([0, 114, 254])
        self.colours.append([0, 216, 255])
        self.colours.append([0, 255, 89])
        self.colours.append([255, 67, 0])
        self.colours.append([255, 0, 170])
        self.colours.append([255, 0, 255])
        self.coloursIndex = 0
        self.numRobotPaths = 0

    def get_box(self, boxIndex, boxType):
        if boxType == 'obs':
            return self.obstacles[boxIndex]
        elif(boxType == 'box'):
            return self.boxes[boxIndex]
        else:
            print("invalid box type: ", boxType)
            exit(0)

    def update_boxes(self):

        for box in self.boxes:
            intersect = True
            while intersect is True:
                edge = int(random.random() * 4)
                if edge == 0:
                    box.end = (box.goal[0] + box.width / 2.0, box.goal[1])
                elif edge == 1:
                    box.end = (box.goal[0] - box.width / 2.0, box.goal[1])
                elif edge == 2:
                    box.end = (box.goal[0], box.goal[1] + box.width / 2.0)
                else:
                    box.end = (box.goal[0], box.goal[1] - box.width / 2.0)
                intersect = self.collides_with_point(box.end)
            box.end = box.goal

            intersect = True
            while intersect is True:
                cent = int(random.random() * 4)
                if cent == 0:
                    box.start = (box.centre[0] + box.width / 2.0, box.centre[1])
                elif cent == 1:
                    box.start = (box.centre[0] - box.width / 2.0, box.centre[1])
                elif cent == 2:
                    box.start = (box.centre[0], box.centre[1] + box.width / 2.0)
                else:
                    box.start = (box.centre[0], box.centre[1] - box.width / 2.0)
                intersect = self.collides_with_point(box.start)
            box.start = box.centre

        for box in self.obstacles:
            intersect = True
            while intersect is True:
                cent = int(random.random() * 4)
                if cent == 0:
                    box.start = (box.centre[0] + box.width / 2.0, box.centre[1])
                elif cent == 1:
                    box.start = (box.centre[0] - box.width / 2.0, box.centre[1])
                elif cent == 2:
                    box.start = (box.centre[0], box.centre[1] + box.width / 2.0)
                else:
                    box.start = (box.centre[0], box.centre[1] - box.width / 2.0)
                intersect = self.collides_with_point(box.centre)
            box.start = box.centre

    def update_goal(self, sample):

        width = self.boxes[0].width / 2.0

        # Update goal location to be on edge of box
        intersect = True
        while intersect is True:
            cent = int(random.random() * 4)
            if cent == 0:
                out = (sample[0] + width, sample[1])
            elif cent == 1:
                out = (sample[0] - width, sample[1])
            elif cent == 2:
                out = (sample[0], sample[1] + width)
            else:
                out = (sample[0], sample[1] - width)
            intersect = self.collides_with_point(sample)
        return sample

    def colours_next(self):
        col = self.colours[self.coloursIndex]
        self.coloursIndex += 1
        self.coloursIndex %= 7
        return col

    def get_object_index(self, centre):
        for i, e in enumerate(self.boxes):
            if e.centre == centre:
                return i, 'box'
        for i, e in enumerate(self.obstacles):
            if e.centre == centre:
                return i, 'obs'
        return -1

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
        self.treeIDs.append(id(tree))

    def box_collision(self, candidateBox, startIgnore, endIgnore, robot=False):

        if robot:
            dummy = endIgnore
            endIgnore = startIgnore
            startIgnore = dummy

        if startIgnore is None:
            startIgnore = (0, 0)

        if endIgnore is None:
            endIgnore = (0, 0)

        for box in self.boxes:
            if box.collides_with_box(candidateBox) and \
                    box.goal[0] != endIgnore[0] and box.goal[1] != endIgnore[1] and \
                    box.start[0] != startIgnore[0] and box.start[1] != startIgnore[1]:
                #print('box start ', box.start, '  box end ', box.end, '  box startIgnore ', startIgnore, '  box endIgnore ', endIgnore)
                return True

        for box in self.obstacles:
            if box.collides_with_box(candidateBox) and \
                    box.start[0] != startIgnore[0] and box.start[1] != startIgnore[1]:

                return False
        return False

    def box_collision_point(self, point, getBox=False):

        for box in self.boxes:
            if box.collides_with_point(point):
                if getBox:
                    return box.centre
                else:
                    return True

        for box in self.obstacles:
            if box.collides_with_point(point):
                if getBox:
                    return box.centre
                else:
                    return True
        return False

    def get_proximal_freedom(self, point):
        w = self.boxes[0].height / 2
        h = self.boxes[0].height / 2
        x = point[0]
        y = point[1]
        return [
            self.box_collision_point((x - w, y)),       # L
            self.box_collision_point((x - w, y + h)),   # LD
            self.box_collision_point((x, y + h)),       # D
            self.box_collision_point((x + w, y + h)),   # RD
            self.box_collision_point((x + w, y)),       # R
            self.box_collision_point((x + w, y - h)),   # RU
            self.box_collision_point((x, y - h)),       # U
            self.box_collision_point((x - w, y - h)),   # LU
        ]

    def box_intersection(self, line, boxes):

        intersect = []
        for box in boxes:
            # print("INTERSECT BOX: ", box.centre)
            if box.collides_with_line(line[0], line[1]):
                intersect.append(utils.get_intersect(box.get_tl(), box.get_tr(), line[0], line[1]))
                intersect.append(utils.get_intersect(box.get_tr(), box.get_br(), line[0], line[1]))
                intersect.append(utils.get_intersect(box.get_br(), box.get_bl(), line[0], line[1]))
                intersect.append(utils.get_intersect(box.get_bl(), box.get_tl(), line[0], line[1]))

        if len(intersect) > 1:

            # Strip out the two remaining points and remove the one that is furthest away from average of line segments
            out = []
            for point in intersect:
                if point:
                    out.append(point)
            return utils.closest_point(out[0], out[1], ((line[0][0] + line[1][0]) / 2, (line[0][1] + line[1][1]) / 2))
        else:
            return None

    def goal_collision(self, candidateBox, ignoreGoal):

        if ignoreGoal is None:
            ignoreGoal = (0, 0)

        for box in self.boxes:
            if Prism(box.width, box.height, box.goal).collides_with_box(candidateBox) and \
                    box.goal[0] != ignoreGoal[0] and box.goal[1] != ignoreGoal[1]:
                return True

        for box in self.obstacles:
            if box.obstacleGoal:
                if Prism(box.width, box.height, box.obstacleGoal).collides_with_box(candidateBox) and \
                        box.obstacleGoal[0] != ignoreGoal[0] and box.obstacleGoal[1] != ignoreGoal[1]:
                    return True
        return False

    def static_collision(self, box):
        for static in self.statics:
            if static.collides_with_box(box):
                return True

    def collides_with_point(self, point):

        for static in self.statics:
            if static.collides_with_point(point):
                return True

    def get_features(self):
        objects = []
        for static in self.statics:
            objects.append(static)

        for box in self.boxes:
            objects.append(box)
            objects.append(Prism(box.width, box.height, box.goal))

        for box in self.obstacles:
            objects.append(box)
            if box.obstacleGoal:
                objects.append(Prism(box.width, box.height, box.obstacleGoal))

        # add world edges
        objects.append(Prism(1.0, 1.0, (0.5, 0.5)))
        return objects

    def sample(self):
        # set the clearance so that you can wedge your thumb between a box centred at sample point and world edge [0, 1]
        bw = self.boxes[0].width

        while True:
            samplePoint = random.random() * (1 - bw - 2 * self.primitive) + bw / 2.0 + 0.001, \
                          random.random() * (1 - bw - 2 * self.primitive) + bw / 2.0 + 0.001

            return samplePoint

    def update_canvas(self):
        self.canvas = np.ones((1000, 1000, 3), dtype=np.uint8) * 255

        for box in range(0, len(self.boxes)):
            self.canvas = vis.plot_box(self.canvas, [[255, 0, 0], [0, 0, 255]],
                                       self.boxes[box].get_tl(),
                                       self.boxes[box].get_br(),
                                       self.boxes[box].goal,
                                       self.boxes[box].path)

        for box in range(0, len(self.obstacles)):
            self.canvas = vis.plot_box(self.canvas, [[255, 0, 0], [0, 0, 255]],
                                       self.obstacles[box].get_tl(),
                                       self.obstacles[box].get_br(),
                                       self.obstacles[box].obstacleGoal,
                                       self.obstacles[box].path,
                                       obstacle=True)

        for box in range(0, len(self.statics)):
            self.canvas = vis.plot_box(self.canvas, [[0, 0, 0], [0, 0, 0]],
                                      self.statics[box].get_tl(),
                                      self.statics[box].get_br(),
                                      None,
                                      None)
        self.canvas = vis.plot_path(self.canvas, self.agent.finalPath)

    def show(self, treeID):

        if treeID is not False:

            for node in iter(self.trees[treeID]):

                # Plot node position
                canvas = vis.plot_sample(self.canvas, node.start)

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

    def get_tl(self):
        return (self.centre[0] - self.width/2), (self.centre[1] - self.width/2)

    def get_tr(self):
        return (self.centre[0] + self.width/2), (self.centre[1] - self.width/2)

    def get_bl(self):
        return (self.centre[0] - self.width/2), (self.centre[1] + self.height/2)

    def get_br(self):
        return (self.centre[0] + self.width/2), (self.centre[1] + self.height/2)

    def get_b(self):
        return self.centre[0], (self.centre[1] + self.height/2)

    def get_t(self):
        return self.centre[0], (self.centre[1] - self.height/2)

    def get_r(self):
        return (self.centre[0] + self.width/2), self.centre[1]

    def get_l(self):
        return (self.centre[0] - self.width/2), self.centre[1]

    def collides_with_box(self, box):
        return abs(self.centre[0] - box.centre[0]) * 2 < (self.width + box.width) and \
               abs(self.centre[1] - box.centre[1]) * 2 < (self.height + box.height)

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
        self.centre = centre
        self.goal = goal
        self.start = None
        self.end = None
        self.sideLength = side
        self.path = None
        self.proximal = None

    def add_path(self, path):
        self.path = path
