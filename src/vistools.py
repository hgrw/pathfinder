import cv2
import src.utils as utils

def colours(i):
    colours = []
    colours.append([0, 0, 255])
    colours.append([0, 114, 254])
    colours.append([0, 216, 255])
    colours.append([0, 255, 89])
    colours.append([255, 67, 0])
    colours.append([255, 0, 170])
    colours.append([255, 0, 255])
    return colours[i]


def animate_path(agent, canvas, path):

    for vertex in path:
        pt1, pt2 = utils.robot_line(agent.width, vertex)

        # print(pt1, pt2)
        # print((int(pt1[0] * 1000), int(pt1[1] * 1000)), (int(pt2[0] * 1000), int(pt2[1] * 1000)))
        cv2.imshow('environment', cv2.circle(cv2.line(canvas,
                                                      (int(pt1[0] * 1000), int(pt1[1] * 1000)),
                                                      (int(pt2[0] * 1000), int(pt2[1] * 1000)), [0, 0, 0], 2),
                                             (int(vertex[0][0] * 1000), int(vertex[0][1] * 1000)), 5, [0, 0, 0], -1))
        cv2.waitKey(0)


def plot_box(canvas, colours, tl, br, goal, path, obstacle=False):

    if obstacle:
        colours[0] = [125, 0, 0]
        colours[1] = [0, 0, 125]

    canvas = cv2.rectangle(canvas, (int(tl[0] * 1000), int(tl[1] * 1000)), (int(br[0] * 1000), int(br[1] * 1000)), colours[0], -1)

    if goal is not None:  # Plot goal too!
        width = int(br[0] * 1000) - int(tl[0] * 1000)
        goal = (int(goal[0] * 1000), int(goal[1] * 1000))
        canvas = cv2.rectangle(canvas, (goal[0] - int(width/2), goal[1] - int(width/2)),
                               (goal[0] + int(width/2), goal[1] + int(width/2)), [255, 255, 0], -1)
        canvas = cv2.circle(canvas, goal, 30, colours[1], -1)

    if path is not None:
        for point in range(1, len(path)):
            cv2.imshow('environment',
                       cv2.line(canvas,
                                (int(path[point - 1][0][0] * 1000), int(path[point - 1][0][1] * 1000)),
                                (int(path[point][0][0] * 1000), int(path[point][0][1] * 1000)), [0, 0, 255], 2))


    return canvas

def plot_path(canvas, paths):

    for path in range(0, len(paths)):
        if len(paths[path]) > 0:
            for point in range(1, len(paths[path])):
                cv2.imshow('environment',
                           cv2.line(canvas,
                                    (int(paths[path][point - 1][0][0] * 1000), int(paths[path][point - 1][0][1] * 1000)),
                                    (int(paths[path][point][0][0] * 1000), int(paths[path][point][0][1] * 1000)), colours(path % 7), 2))
    return canvas


def plot_sample(canvas, sample):
    return cv2.circle(canvas, (int(sample[0] * 1000), int(sample[1] * 1000)), 5, [99, 99, 99], -1)
