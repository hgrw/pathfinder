
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
