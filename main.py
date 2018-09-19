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

def main(input, output, demo=False, debug=False):

    env = io.initialise_environment(input)
    env.update_boxes()


    # Clearance parameters
    robotClearance = env.primitive
    boxClearance = env.boxes[0].width
    debug=False

    # Set stepsize as min(box width, static obstacle width)
    stepSize = env.boxes[0].width

    # 1. Generate paths for each box, ignoring movable obstacles and other movable boxes.
    for box in range(0, len(env.boxes)):

        env.currentGoal = env.boxes[box].goal
        env.boxes[box].add_path(
            tree.generate_path(env, env.boxes[box].start, env.boxes[box].end, stepSize, boxClearance, plot=debug))

    # 2. Identify free space for movable obstacles and generate associated goal locations.
    for box in range(0, len(env.obstacles)):

        # Add goal location for obstacle
        env.obstacles[box].obstacleGoal = agent.generate_goal(env)
        env.obstacles[box].end = env.update_goal(env.obstacles[box].obstacleGoal)

        # 3. Generate paths for getting movable obstacles to goal locations.
        env.obstacles[box].add_path(
            tree.generate_path(env,
                               env.obstacles[box].start,
                               env.obstacles[box].end,
                               stepSize, boxClearance, plot=debug))

    # 4. Get robot to start location
    env.agent.finalPath.append(
        tree.generate_path(env,
                           (env.agent.x, env.agent.y),
                           env.obstacles[0].start,
                           stepSize, robotClearance,
                           plot=debug, robot=True))

    # 5. If there are more than one obstacle, generate path between goal of preceeding obstacle to start of next
    for obs in range(1, len(env.obstacles)):
        env.agent.finalPath.append(env.obstacles[obs - 1].path)
        env.agent.finalPath.append(
            tree.generate_path(env,
                               env.obstacles[obs - 1].end,
                               env.obstacles[obs].start,
                               stepSize, robotClearance,
                               plot=debug, robot=True))

    # 5.5. If we only have one obstacle, just add the trajectory to agent path, hence previous for loop FAILED
    if len(env.obstacles) == 1:
        # Add obstacle start to goal path for each obstacle
        for obstacle in env.obstacles:
            env.agent.finalPath.append(obstacle.path)

    # 6. Get robot from last obstacle goal to first box start
    env.agent.finalPath.append(
        tree.generate_path(env,
                           env.obstacles[-1].end,
                           env.boxes[0].start,
                           stepSize, robotClearance,
                           plot=debug, robot=True))

    # 6.5 If there are more than one box, generate path between goal of preceeding box to start of next
    for box in range(1, len(env.boxes)):
        env.agent.finalPath.append(env.boxes[box - 1].path)
        env.agent.finalPath.append(
            tree.generate_path(env,
                               env.boxes[box - 1].end,
                               env.boxes[box].start,
                               stepSize, robotClearance,
                               plot=debug, robot=True))

    # If we only have one box, just add the trajectory to agent path, hence previous for loop FAILED
    if len(env.boxes) == 1:
        for box in env.boxes:
            env.agent.finalPath.append(box.path)
    else:
        env.agent.finalPath.append(env.boxes[-1].path)

    env.update_canvas()

    cv2.waitKey(0)

    env.agent.extrapolate_path(env, env.agent.finalPath)

    env.update_canvas()
    cv2.imshow('environment', env.canvas)
    cv2.waitKey(0)
    # 5. Generate paths between movable boxes
    currentPos = env.trees[env.treeIDs[-2]].goal

    #
    # for pos in range(2, len(env.treeIDs)):
    #
    #     print("Traversing end-start gap")
    #     traversalOrder = env.treeIDs.reverse()
    #     env.agent.finalPath.append(
    #         tree.generate_path(env,
    #                            currentPos,
    #                            env.trees[env.treeIDs[-pos]].start,
    #                            stepSize, boxClearance,
    #                            plot=debug))
    #     currentPos = env.trees[env.treeIDs[-pos]].goal



    # 4. Modify paths to include robot subroutines at corners
    # env.agent.boxPaths = agent.update_path(env, env.agent.boxPaths)
    # env.agent.obstPaths = agent.update_path(env, env.agent.obstPaths)

    # 5. Generate path for robot to go to start location to moving obstacles.





    # 7. Generate path for robot to go from end of each box path to start of next moving box path.



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='COMP3702 A1: provide input text file.')
    parser.add_argument('input', metavar='input %s', help='path to input file')
    parser.add_argument('output', metavar='output %s', help='path to save output file.')

    parser.add_argument('--debug', default=0, action='store_true',
                        help='Plot each trajectory')

    parser.add_argument('--demo', default=0, action='store_true',
                        help='Plot mode')

    print('ok')


args = parser.parse_args()

main(args.input, args.output, args.debug, args.demo)