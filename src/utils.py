
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
