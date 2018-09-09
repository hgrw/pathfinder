import cv2
import numpy as np
import argparse
import sys
sys.path.append("./src")
import src.io as io
import src.tree as tree
import src.agent as agent
import src.utils as utils

import src.vistools as vis

from src.environment import Env, Prism, Moving

"""

1. Generate paths for each movable box, ignoring movable obstacles and other movable boxes.
2. Identify free space for movable obstacles and generate associated goal locations.
3. Generate paths for getting movable obstacles to goal locations.
4. Modify paths to include robot subroutines at corners
5. Generate path for robot to go to start location to moving obstacles.
6. Generate path order-of-execution
7. Generate path for robot to go from end of each box path to start of next moving box path.


"""

def main(input, output, demo=False):

    # Parameters
    stepSize = 0.08
    boxClearance = 1.0

    env = io.initialise_environment(input)

    # 1. Generate paths for each box, ignoring movable obstacles and other movable boxes.
    for box in range(0, len(env.boxes)):

        env.agent.add_box_path(
            tree.generate_path(env, env.boxes[box].centre, env.boxes[box].goal, stepSize, boxClearance, plot=demo))

    # 2. Identify free space for movable obstacles and generate associated goal locations.
    for box in range(0, len(env.obstacles)):

        # If obstacle lay on path for any box,
        if agent.box_intersect_path(env.agent.boxPaths, env.obstacles[box]):

            # Add goal location for obstacle
            env.obstacles[box].add_goal(agent.generate_goal(env))

            # 3. Generate paths for getting movable obstacles to goal locations.
            env.agent.add_obst_path(
                tree.generate_path(env,
                                   env.obstacles[box].centre,
                                   env.obstacles[box].obstacleGoal,
                                   stepSize, boxClearance, plot=demo))

    # 4. Modify paths to include robot subroutines at corners
    env.agent.boxPaths = env.agent.update_paths(env.agent.boxPaths)
    env.agent.obstPaths = env.agent.update_paths(env.agent.obstPaths)

    # 5. Generate path for robot to go to start location to moving obstacles.


    # 6. Generate path order-of-execution


    # 7. Generate path for robot to go from end of each box path to start of next moving box path.



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='COMP3702 A1: provide input text file.')
    parser.add_argument('input', metavar='input %s', help='path to input file')
    parser.add_argument('output', metavar='output %s', help='path to save output file.')

    parser.add_argument('--demo', default=0, action='store_true',
                        help='Plot mode')

    print('ok')


args = parser.parse_args()

main(args.input, args.output, args.demo)