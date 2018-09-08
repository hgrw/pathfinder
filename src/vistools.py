import cv2


def plot_box(canvas, dimms, colours, tl, br, goal=False, obstacle=False):

    if obstacle:
        colours[0] = [125, 0, 0]
        colours[1] = [0, 0, 125]

    if dimms[0] != 1:
        tl = (int(tl[0]), int(tl[1]))
        br = (int(br[0]), int(br[1]))
    canvas = cv2.rectangle(canvas, tl, br, colours[0], -1)

    if goal is not False:  # Plot goal too!
        goal = (int(goal[0]), int(goal[1]))
        canvas = cv2.circle(canvas, goal, 30, colours[1], -1)

    return canvas


def plot_sample(canvas, sample, dimms):

    if dimms[0] != 1:
        sample = (int(sample[0] * dimms[0]), int(sample[1] * dimms[0]))

    return cv2.circle(canvas, sample, 5, [99, 99, 99], -1)
