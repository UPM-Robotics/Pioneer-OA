#!/home/javinator9889/.pyenv/shims/python
# /usr/bin/python3

print('### Script:', __file__)

import os
import sys
import math

import vrep

import pickle

import numpy as np
from time import sleep
import matplotlib.pyplot as plt

from mapper import Mapper
from motion import Motion
from planner import Planner
from map_wrapper import Wrapper
from map_printer import start_printing


# from .robot_control import Sensor

# --------------------------------------------------------------------------

def getRobotHandles(clientID):
    # Robot handle
    _, rbh = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx',
                                      vrep.simx_opmode_blocking)

    # Motor handles
    _, lmh = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor',
                                      vrep.simx_opmode_blocking)
    _, rmh = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor',
                                      vrep.simx_opmode_blocking)

    # Sonar handles
    str = 'Pioneer_p3dx_ultrasonicSensor%d'
    sonar = [0] * 16
    for i in range(16):
        _, h = vrep.simxGetObjectHandle(clientID, str % (i + 1),
                                        vrep.simx_opmode_blocking)
        sonar[i] = h
        vrep.simxReadProximitySensor(clientID, h, vrep.simx_opmode_streaming)

    return [lmh, rmh], sonar, rbh


# --------------------------------------------------------------------------

def setSpeed(clientID, hRobot, lspeed, rspeed):
    vrep.simxSetJointTargetVelocity(clientID, hRobot[0][0], lspeed,
                                    vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetVelocity(clientID, hRobot[0][1], rspeed,
                                    vrep.simx_opmode_oneshot)


# --------------------------------------------------------------------------

def getSonar(clientID, hRobot):
    r = [1.0] * 16
    for i in range(16):
        handle = hRobot[1][i]
        e, s, p, _, _ = vrep.simxReadProximitySensor(clientID, handle,
                                                     vrep.simx_opmode_buffer)
        if e == vrep.simx_return_ok and s:
            r[i] = math.sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2])

    return r


# --------------------------------------------------------------------------

def getRobotPosition(clientID, hRobot):
    _, rpos = vrep.simxGetObjectPosition(clientID, hRobot[2], -1,
                                         vrep.simx_opmode_streaming)
    return rpos[0:2]


# --------------------------------------------------------------------------

def getRobotHeading(clientID, hRobot):
    _, reul = vrep.simxGetObjectOrientation(clientID, hRobot[2], -1,
                                            vrep.simx_opmode_streaming)
    return reul[2]


# --------------------------------------------------------------------------
# --------------------------------------------------------------------------
clr = "                                                                                "


def avoid(robot):
    if not robot.is_any_obstacle_front():
        # print(clr, end="\r")
        # print(" >>> Looking for wall", end="\r")
        if any(x < 10 for x in robot.sensors.parallel_left):
            # print(clr, end="\r")
            # print(" ! Obstacle parallel to the left side", end="\r")
            if robot.sensors.parallel_left[0] < robot.sensors.parallel_left[1]:
                lspeed, rspeed = 0.25, 0
            elif robot.sensors.parallel_left[0] == robot.sensors.parallel_left[
                1]:
                lspeed, rspeed = 1, 1
            else:
                lspeed, rspeed = 0.25, 1

        elif any(x < 10 for x in robot.sensors.parallel_right):
            # print(clr, end="\r")
            # print(" ! Obstacle parallel to the right side", end="\r")
            if robot.sensors.parallel_right[0] < \
                    robot.sensors.parallel_right[1]:
                lspeed, rspeed = 0, 0.5

            else:
                lspeed, rspeed = 1, 1
        else:
            lspeed, rspeed = 0.25, 1

    else:
        if robot.is_any_obstacle_right():
            # print(clr, end="\r")
            # print("! Obstacle right", end="\r")
            if robot.is_any_obstacle_left():
                # print(clr, end="\r")
                # print("! Obstacle left", end="\r")
                lspeed, rspeed = 10, -10
            else:
                lspeed, rspeed = 0.1, 1
        elif robot.is_any_obstacle_left():
            # print(clr, end="\r")
            # print("! Obstacle left", end="\r")
            if robot.is_any_obstacle_right():
                lspeed, rspeed = -10, 10
            else:
                lspeed, rspeed = 1, 0.1
        else:
            # print(clr, end="\r")
            # print(" >>> Looking for wall", end="\r")
            lspeed, rspeed = 2, 2

    return lspeed, rspeed


# --------------------------------------------------------------------------

def main():
    print('### Program started')

    print('### Number of arguments:', len(sys.argv), 'arguments.')
    print('### Argument List:', str(sys.argv))

    vrep.simxFinish(-1)  # just in case, close all opened connections

    port = int(sys.argv[1])
    clientID = vrep.simxStart('127.0.0.1', port, True, True, 2000, 5)

    if clientID == -1:
        print('### Failed connecting to remote API server')

    else:
        print('### Connected to remote API server')
        hRobot = getRobotHandles(clientID)
        # Recover later generated data
        if os.path.exists("map.cls"):
            try:
                with open("map.cls", "rb") as created_map:
                    map_wrapper = pickle.load(created_map)
            except Exception as _:
                robot = Mapper(X0=-2,
                               Y0=2,
                               map_width=4,
                               map_height=4,
                               grid_size=(26, 26),
                               k=1.0,
                               initial_threshold=0.4,
                               max_read_distance=0.5,
                               ratio=1E-6)
            else:
                robot = map_wrapper.restore()
        else:
            robot = Mapper(X0=-2,
                           Y0=2,
                           map_width=4,
                           map_height=4,
                           grid_size=(26, 26),
                           k=1.0,
                           initial_threshold=0.4,
                           max_read_distance=0.5,
                           ratio=1E-6)
        sensors = robot.sensors
        printer = start_printing(robot.lock)

        while vrep.simxGetConnectionId(clientID) != -1:
            # Perception
            sonar = getSonar(clientID, hRobot)
            x, y = getRobotPosition(clientID, hRobot)

            heading = getRobotHeading(clientID, hRobot)
            sensors.set_sonar(sonar)

            # Planning
            lspeed, rspeed = avoid(robot)

            # Action
            setSpeed(clientID, hRobot, lspeed, rspeed)
            # Update robot position and map location
            robot.update_robot_position(x, y,
                                        np.asarray(sonar, dtype=np.float_),
                                        heading)
            print(clr, end="\r")
            print(f"Threshold: {robot.threshold}", end="\r")
            # time.sleep(0.1)

        print('### Finishing...')
        vrep.simxFinish(clientID)

        # As we are using Cython, use a wrapper for saving it into a file
        wrapper = Wrapper(robot)
        with open("map.cls", "wb") as file_map:
            pickle.dump(wrapper, file_map, protocol=pickle.HIGHEST_PROTOCOL)

        # Close the process we have just created for printing in real-time
        printer.terminate()
        printer.join()
        printer.close()

        figure = plt.figure(figsize=(6, 6))
        axis = figure.add_subplot(111)
        image = axis.imshow(np.random.randint(0, 10, size=robot.grid.shape),
                            cmap="gray_r")

        annotations_grid = np.zeros(robot.grid.shape)

        plt.show(block=False)
        image.set_data(robot.grid)
        for index in np.ndindex(robot.grid.shape):
            if robot.threshold < robot.grid[index] != annotations_grid[index]:
                axis.annotate('■',
                              xy=(index[1], index[0]),
                              horizontalalignment="center",
                              verticalalignment="center",
                              color="black",
                              size=4)
            annotations_grid[index] = robot.grid[index]
        figure.canvas.draw_idle()
        plan = Planner(robot)
        path = plan.calculate_path((-1, -1), (1, -1))
        axis.annotate('■',
                      xy=(path[0][1], path[0][0]),
                      horizontalalignment="center",
                      verticalalignment="center",
                      color="green",
                      size=8)
        for index in plan.calculate_path((-1, -1), (1, -1)):
            axis.annotate('■',
                          xy=(index[1], index[0]),
                          horizontalalignment="center",
                          verticalalignment="center",
                          color="blue",
                          size=4)
            figure.canvas.draw_idle()
            plt.pause(0.3)
        axis.annotate('■',
                      xy=(path[len(path) - 1][1], path[len(path) - 1][0]),
                      horizontalalignment="center",
                      verticalalignment="center",
                      color="orange",
                      size=16)
        plt.pause(0.01)
        plt.show()

        del robot

    print('### Program ended')


# --------------------------------------------------------------------------

if __name__ == '__main__':
    main()
