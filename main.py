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
2. Modify paths to include sub-path for robot to change push-location on movable boxes.
3. Identify regions where paths intersect. For each intersection, either
- redraw paths; or
- identify order-of-execution that works (neither of these solutions are that great tbh...surely there's a better way)
4. Identify free space for movable obstacles, hence generate goal locations.
5. Generate paths for getting movable obstacles to goal locations.
6. Modify paths to include sub-path for robot to change push-location on movable obstacles.
7. Identify regions where movable obstacle paths intersect and fix as per moving boxes (redraw or identify order-of-execution).
8. Generate path for robot to go to start location for moving obstacles.
9. Generate path for robot to go from end of each moving obstacle path to start of next moving obstacle path.
10. Generate path for robot to go from final moving obstacle goal location to start of first moving box path.
11. Generate path for robot to go from end of each moving box path to start of next moving box path.
12. Done 

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

    for box in range(0, len(env.obstacles)):

        # Add goal location for movable obstacle based on paths in agent.
        # remember, need to ensure that obstacle goals don't intersect!
        env.obstacles[box].add_goal(agent.generate_goal(env))
        env.agent.add_obst_path(
            tree.generate_path(env,
                               env.obstacles[box].centre,
                               env.obstacles[box].obstacleGoal,
                               stepSize, boxClearance, plot=demo))


    # 2. Modify path to include sub-path for robot to change push-location on movable boxes.
    #path = agent.augment_path(env, path)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='COMP3702 A1: provide input text file.')
    parser.add_argument('input', metavar='input %s', help='path to input file')
    parser.add_argument('output', metavar='output %s', help='path to save output file.')

    parser.add_argument('--demo', default=0, action='store_true',
                        help='Plot mode')

    print('ok')


args = parser.parse_args()

main(args.input, args.output, args.demo)