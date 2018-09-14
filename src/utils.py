import math
import numpy as np

def sign(x):
    return (1 - (x <= 0))*2 - 1


def order(p1, p2):

    if p1[0] == p2[0]:  # Vertical
        if p1[1] < p2[1]:
            return p1, p2
        else:
            return p2, p1
    else:
        if p1[0] < p2[0]:
            return p1, p2
        else:
            return p2, p1


def angle(line):
    p1 = line[0]
    p2 = line[1]
    if p1[0] == p2[0]:
        return math.pi/2
    else:
        return 0


def get_face(pt1, pt2):
    if pt1[0] < pt2[0]:
        return 0  # left
    elif pt1[0] == pt2[0]:
        if pt1[1] < pt2[1]:
            return 3  # top
        else:
            return 4  # bottom
    else:
        return 2  # right


def dist_to_static(statics, point, agentWidth):
    for static in statics:

        lineDist = []
        cornerDist = []
        corners = [static.get_tl(), static.get_tr(), static.get_br(), static.get_bl()]
        edges = np.asarray([(corners[0], corners[1]),
                 (corners[1], corners[2]),
                 (corners[2], corners[3]),
                 (corners[3], corners[0])])

        for edge in range(0, 4):
            lineDist.append(
                np.cross(edges[edge][1] - edges[edge][0], point - edges[edge][0]) / np.linalg.norm(edges[edge][1] - edges[edge][0]))
            cornerDist.append(math.hypot(point[0] - corners[edge][0], point[1] - corners[edge][1]))

        # Get index of closest edge or corner
        minLine = lineDist.index(min(lineDist))
        minCorner = cornerDist.index(min(cornerDist))

        # We only care about theta if it is going to touch the sides of any static points
        if cornerDist[minCorner] > agentWidth and lineDist[minLine] > agentWidth:
            return None
        else:
            if cornerDist[minCorner] <= lineDist[minLine]:
                direction = math.atan2(corners[minCorner][1] - point[1], corners[minCorner][0] - point[0])
                return cornerDist[minCorner], direction
            else:
                return lineDist[minLine], angle(edges[minLine])