from .environment import Agent, Env, Moving, Prism


def initialise_environment(text):

    doc = []
    with open(text, 'r') as f:
        for line in f:
            doc.append([float(x) for x in line.split(' ')])

    # Get hard-coded stuff from file
    numBoxes = int(doc[1][0])
    numObjects = int(doc[1][1])
    numStatics = int(doc[1][2])
    boxes = doc[2:2 + numBoxes]
    obstacles = doc[2 + numBoxes:2 + numBoxes + numObjects]
    statics = doc[2 + numBoxes + numObjects: 2 + numBoxes + numObjects + numStatics]
    boxWidth = obstacles[0][2]
    env = Env()

    # Add boxes to environment
    for box in range(0, numBoxes):
        env.add_box(Moving(boxWidth, (boxes[box][0], boxes[box][1]), (boxes[box][2], boxes[box][3])))

    # Add movable obstacles to environment
    for box in range(0, numObjects):
        env.add_obst(Moving(boxWidth, (obstacles[box][0], obstacles[box][1]), (0, 0)))

    # Add static obstacles to environment
    for box in range(0, numStatics):
        width = statics[box][2] - statics[box][0]
        height = statics[box][3] - statics[box][1]
        centre = ((statics[box][0] + statics[box][2])/2.0, (statics[box][1] + statics[box][3])/2.0)
        env.add_static(Prism(width, height, centre))

    # Add agent to environment
    env.add_agent(Agent(doc[0][0], doc[0][1], doc[0][2], doc[0][3]))

    return env