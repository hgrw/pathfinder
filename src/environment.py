import numpy as np
import cv2
import random

class Env(object):

    statics = []

    def __init__(self):
        self.boxes = []
        self.obstacles = []
        self.agent = None

    def add_box(self, box):
        self.boxes.append(box)

    def add_obst(self, obstacle):
        self.obstacles.append(obstacle)

    def add_static(self, static):
        self.statics.append(static)

    def add_agent(self, agent):
        self.agent = agent

    def sample(self):
        return random.random(), random.random()

    def show(self, dimms):
        canvas = np.ones(dimms, dtype=np.uint8) * 255

        for box in range(0, len(self.boxes)):
            tl = self.boxes[box].get_tl(dimms[0])
            br = self.boxes[box].get_br(dimms[0])
            goal = self.boxes[box].get_goal(dimms[0])
            if dimms[0] != 1:
                tl = (int(tl[0]), int(tl[1]))
                br = (int(br[0]), int(br[1]))
                goal = (int(goal[0]), int(goal[1]))
            canvas = cv2.rectangle(canvas, tl, br, [255, 0, 0], -1)
            canvas = cv2.circle(canvas, goal, 30, [0, 0, 255], -1)

        for box in range(0, len(self.obstacles)):
            tl = self.obstacles[box].get_tl(dimms[0])
            br = self.obstacles[box].get_br(dimms[0])
            if dimms[0] != 1:
                tl = (int(tl[0]), int(tl[1]))
                br = (int(br[0]), int(br[1]))
            canvas = cv2.rectangle(canvas, tl, br, [0, 0, 0], -1)

        for box in range(0, len(self.statics)):
            tl = self.statics[box].get_tl(dimms[0])
            br = self.statics[box].get_br(dimms[0])
            if dimms[0] != 1:
                tl = (int(tl[0]), int(tl[1]))
                br = (int(br[0]), int(br[1]))
            canvas = cv2.rectangle(canvas, tl, br, [0, 0, 0], -1)

        return canvas

class Prism(object):

    def __init__(self, width, height, centre):
        self.width = width
        self.height = height
        self.centre = centre

    def get_tl(self, scale):
        return (self.centre[0] - self.width/2) * scale, (self.centre[1] - self.width/2) * scale

    def get_tr(self, scale):
        return (self.centre[0] + self.width/2) * scale, (self.centre[1] - self.width/2) * scale

    def get_bl(self, scale):
        return (self.centre[0] - self.width/2) * scale, (self.centre[1] + self.height/2) * scale

    def get_br(self, scale):
        return (self.centre[0] + self.width/2) * scale, (self.centre[1] + self.height/2) * scale


class Moving(Prism):

    def __init__(self, side, centre, goal):
        super().__init__(side, side, centre)
        self.sideLength = side
        self.centre = centre
        self.goal = goal

    def get_goal(self, scale):
        return (self.goal[0] - self.sideLength/2) * scale, (self.goal[1] - self.sideLength/2) * scale

class Agent(object):

    def __init__(self, width, x, y, theta):
        self.width = width
        self.x = x
        self.y = y
        self.theta = theta