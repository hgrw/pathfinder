import cv2
import numpy as np
import argparse
import sys
sys.path.append("./src")
import src.io as io
import src.tree as tree
import src.vistools as vis

from src.environment import Env, Prism, Moving

def main(input, output, demo=0):

    env = io.initialise_environment(input)

    cv2.imshow('environment', env.show((1000, 1000, 3)))


    for box in range(0, len(env.boxes)):
        path = tree.generate_path(env, env.boxes[box].centre, env.boxes[box].goal, 0.1)



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='COMP3702 A1: provide input text file.')
    parser.add_argument('input', metavar='input %s', help='path to input file')
    parser.add_argument('output', metavar='output %s', help='path to save output file.')

    parser.add_argument('--demo', default=0, action='store_true',
                        help='Plot mode')

    print('ok')


args = parser.parse_args()

main(args.input, args.output, args.demo)