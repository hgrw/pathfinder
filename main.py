import cv2
import numpy as np
import argparse
import sys
sys.path.append("./src")
import src.io as io
import src.tree as tree

import src.vistools as vis

from src.environment import Env, Prism, Moving

def main(input, output, demo=False):

    env = io.initialise_environment(input)
    canvas = env.show((1000, 1000, 3))

    # Generate tree and solve for path for each box
    for box in range(0, len(env.boxes)):
        path = tree.generate_path(env, env.boxes[box].centre, env.boxes[box].goal, 0.08, plot=0)
        #cv2.waitKey(0)



        # plot path
        for point in range(1, len(path)):
            canvas = cv2.line(canvas,
                              (int(path[point - 1][0] * 1000), int(path[point - 1][1] * 1000)),
                              (int(path[point][0] * 1000), int(path[point][1] * 1000)), [0, 0, 0], 2)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='COMP3702 A1: provide input text file.')
    parser.add_argument('input', metavar='input %s', help='path to input file')
    parser.add_argument('output', metavar='output %s', help='path to save output file.')

    parser.add_argument('--demo', default=0, action='store_true',
                        help='Plot mode')

    print('ok')


args = parser.parse_args()

main(args.input, args.output, args.demo)