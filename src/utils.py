import math
import numpy as np
import cv2

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
    if p1[1] == p2[1]:
        return math.pi/2
    elif p1[0] == p2[0]:
        return 0
    else:
        return math.atan2(p1[1] - p2[1], p1[0] - p2[0])


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

def scale(point):
    return (int(point[0] * 1000), int(point[1] * 1000))


def robot_line(width, vertex):
    # print(width, vertex)
    # print((vertex[0][0] - (width / 2) * math.cos(vertex[1][1]), vertex[0][1] - (width / 2) * math.sin(vertex[1][1])),
    #        (vertex[0][0] + (width / 2) * math.cos(vertex[1][1]), vertex[0][1] + (width / 2) * math.sin(vertex[1][1])))

    return (vertex[0][0] - (width / 2) * math.cos(vertex[1]), vertex[0][1] - (width / 2) * math.sin(vertex[1])), \
           (vertex[0][0] + (width / 2) * math.cos(vertex[1]), vertex[0][1] + (width / 2) * math.sin(vertex[1]))


def closest_point(p1, p2, p3):

    # Get closest point from p1, p2, to pe3
    # print(p1, p2, p3)
    # print(math.hypot(p1[0] - p3[0], p1[1] - p3[1]))
    # print(math.hypot(p2[0] - p3[0], p2[1] - p3[1]))
    # cv2.waitKey(0)
    if math.hypot(p1[0] - p3[0], p1[1] - p3[1]) > math.hypot(p2[0] - p3[0], p2[1] - p3[1]):
        return p2
    else:
        return p1


def plot_robot(canvas, width, configuration, colour):
    pt1, pt2 = robot_line(width, configuration)
    cv2.imshow('environment',
               cv2.circle(cv2.line(canvas,
                                   scale(pt1),
                                   scale(pt2), colour, 2),
                          scale(configuration[0]), 5, colour, -1))
    cv2.waitKey(1)


def get_intersect(a1, a2, b1, b2):
    """
    Returns the point of intersection of the lines passing through a2,a1 and b2,b1.
    a1: [x, y] a point on the first line
    a2: [x, y] another point on the first line
    b1: [x, y] a point on the second line
    b2: [x, y] another point on the second line
    """
    s = np.vstack([a1,a2,b1,b2])        # s for stacked
    h = np.hstack((s, np.ones((4, 1)))) # h for homogeneous
    l1 = np.cross(h[0], h[1])           # get first line
    l2 = np.cross(h[2], h[3])           # get second line
    x, y, z = np.cross(l1, l2)          # point of intersection
    if z == 0:                          # lines are parallel
        return None
    return (x/z, y/z)


def add_corners(box, curr, currentFace, nextFace, newGoal):

    default = [
        False,  # 0 L
        False,  # 1 LD
        False,  # 2 D
        False,  # 3 RD
        False,  # 4 R
        False,  # 5 RU
        False,  # 6 U
        False  # 7 LU
    ]
    if sum(currentFace) > 1 or sum(nextFace) > 1:
        print("CURRENT FACE OR NEXT FACE ERROR! ", sum(currentFace), sum(nextFace))
        exit(0)
    elif currentFace.index(True) == nextFace.index(True):    # Current face == next face!
        print("current face == next face")
        return None, 0
    elif currentFace.index(True) == 0:      # Currently on left face
        if nextFace.index(True) == 2:
            return [[box.get_bl(), curr[1], curr[2]],               # Go down
                    [box.get_bl(), curr[1] + math.pi / 2, curr[2]], # Change angle
                    [newGoal, curr[1] + math.pi / 2, curr[2]]], 3   # End at bottom face
        elif nextFace.index(True) == 4:
            return [[box.get_bl(), curr[1], curr[2]],               # Go down
                    [box.get_bl(), curr[1] + math.pi / 2, curr[2]], # Change angle
                    [box.get_br(), curr[1] + math.pi / 2, curr[2]], # Go right
                    [box.get_br(), curr[1] + math.pi, curr[2]],     # Change angle
                    [newGoal, curr[1] + math.pi, curr[2]]], 5       # End on right face
        elif nextFace.index(True) == 6:     # Go up, right
            return [[box.get_tl(), curr[1], curr[2]],               # Go up
                    [box.get_tl(), curr[1] - math.pi / 2, curr[2]], # Change angle
                    [newGoal, curr[1] - math.pi / 2, curr[2]]], 3   # End at top face
        else:
            print("!!!!!!!!!!!!!!!!!!!!!!Next Face error (lays on corner)")
            print(currentFace.index(True))
            print(nextFace.index(True))
            print()
            cv2.waitKey(0)
            exit(0)
    elif currentFace.index(True) == 2:      # Currently on down face
        if nextFace.index(True) == 0:
            return [[box.get_bl(), curr[1], curr[2]],               # Go left
                    [box.get_bl(), curr[1] - math.pi / 2, curr[2]], # Change angle
                    [newGoal, curr[1] - math.pi / 2, curr[2]]], 3   # End at left face
        elif nextFace.index(True) == 6:
            return [[box.get_bl(), curr[1], curr[2]],               # Go left
                    [box.get_bl(), curr[1] - math.pi / 2, curr[2]], # Change angle
                    [box.get_tl(), curr[1] - math.pi / 2, curr[2]], # Go up
                    [box.get_tl(), curr[1] - math.pi, curr[2]],     # Change angle
                    [newGoal, curr[1] + math.pi, curr[2]]], 5       # End at top face
        elif nextFace.index(True) == 4:     # Go up
            return [[box.get_br(), curr[1], curr[2]],               # Go right
                    [box.get_br(), curr[1] + math.pi / 2, curr[2]], # Change angle
                    [newGoal, curr[1] + math.pi / 2, curr[2]]], 3   # End at right face
        else:
            print("!!!!!!!!!!!!!!!!!!Next Face error (lays on corner)")
            print(currentFace.index(True))
            print(nextFace.index(True))
            print()
            cv2.waitKey(0)
            exit(0)

    elif currentFace.index(True) == 4:      # Currently on right face
        if nextFace.index(True) == 2:
            return [[box.get_br(), curr[1], curr[2]],               # Go down
                    [box.get_br(), curr[1] - math.pi / 2, curr[2]], # Change angle
                    [newGoal, curr[1] - math.pi / 2, curr[2]]], 3   # End at bottom face
        elif nextFace.index(True) == 0:
            return [[box.get_tr(), curr[1], curr[2]],               # Go up
                    [box.get_tr(), curr[1] + math.pi / 2, curr[2]], # Change angle
                    [box.get_tl(), curr[1] + math.pi / 2, curr[2]], # Go left
                    [box.get_tl(), curr[1] + math.pi, curr[2]],     # Change angle
                    [newGoal, curr[1] + math.pi, curr[2]]], 5       # End at left face
        elif nextFace.index(True) == 6:     # Go up
            return [[box.get_tr(), curr[1], curr[2]],               # Go up
                    [box.get_tr(), curr[1] + math.pi / 2, curr[2]], # Change angle
                    [newGoal, curr[1] + math.pi / 2, curr[2]]], 3   # End at top face
        else:
            print("!!!!!!!!!!!!!!!!!!!!!!!!Next Face error (lays on corner)")
            print(currentFace.index(True))
            print(nextFace.index(True))
            print()
            cv2.waitKey(0)
            exit(0)

    elif currentFace.index(True) == 6:      # Currently on top face
        if nextFace.index(True) == 0:
            return [[box.get_tl(), curr[1], curr[2]],               # Go left
                    [box.get_tl(), curr[1] + math.pi / 2, curr[2]], # Change angle
                    [newGoal, curr[1] + math.pi / 2, curr[2]]], 3   # End at left face
        elif nextFace.index(True) == 2:
            return [[box.get_tl(), curr[1], curr[2]],               # Go left
                    [box.get_tl(), curr[1] + math.pi / 2, curr[2]], # Change angle
                    [box.get_bl(), curr[1] + math.pi / 2, curr[2]], # Go down
                    [box.get_bl(), curr[1] + math.pi, curr[2]],     # Change angle
                    [newGoal, curr[1] + math.pi, curr[2]]], 5       # End at bottom face
        elif nextFace.index(True) == 4:     # Go up
            return [[box.get_tr(), curr[1], curr[2]],               # Go right
                    [box.get_tr(), curr[1] - math.pi / 2, curr[2]], # Change angle
                    [newGoal, curr[1] - math.pi / 2, curr[2]]], 3   # End at right face
        else:
            print("!!!!!!!!!!!!!!!!!!!!!Next Face error (lays on corner)")
            print(currentFace.index(True))
            print(nextFace.index(True))
            print()
            cv2.waitKey(0)
            exit(0)

    else:
        print('FACE TRANSLATION ERROR! exiting')
        exit(0)


def vector_to_object(objects, point):

    vect = []

    for box in objects:

        corners = [box.get_tl(), box.get_tr(), box.get_br(), box.get_bl()]
        edges = np.asarray([(corners[0], corners[1]),
                            (corners[1], corners[2]),
                            (corners[2], corners[3]),
                            (corners[3], corners[0])])

        # Angle and distance to corners
        for corner in corners:
            x = corner[0] - point[0]
            y = corner[1] - point[1]
            vect.append([math.hypot(x, y), math.atan2(y, x)])
            #print('ok')

        for edge in edges:

            # Does the edge align with x-axis? (i.e. y is within upper and lower corners of box)
            if edge[0][1] < point[1] < edge[1][1]:  # rhs
                vect.append([abs(edge[0][0] - point[0]), 0])
            elif edge[1][1] < point[1] < edge[0][1]:    # lhs
                vect.append([abs(edge[0][0] - point[0]), math.pi])

            # Does the edge align with y-axis? (i.e. x is within upper and lower corners of box)
            elif edge[0][0] < point[0] < edge[1][0]:    # top
                vect.append([abs(edge[0][1] - point[1]), math.pi / 2])
            elif edge[1][0] < point[0] < edge[0][0]:    # bottom
                vect.append([abs(edge[0][1] - point[1]), 3 * math.pi / 2])

            # Corner will be closer than edge in this case
            else:
                pass

    distances = [v[0] for v in vect]
    return vect[distances.index(min(distances))]
