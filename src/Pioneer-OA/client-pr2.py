#!/home/javinator9889/.pyenv/shims/python
# /usr/bin/python3

print('### Script:', __file__)

import math
import sys
import time

import numpy as np
import vrep

from robot_control import PioneerMap
import matplotlib.pyplot as plt


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
        print(clr, end="\r")
        print(" >>> Going forward", end="\r")
        if any(x < 10 for x in robot.sensors.parallel_left):
            print(clr, end="\r")
            print(" ! Obstacle parallel to the left side", end="\r")
            if robot.sensors.parallel_left[0] < robot.sensors.parallel_left[1]:
                lspeed, rspeed = 0.5, 0
            else:
                lspeed, rspeed = 1, 1
        elif any(x < 10 for x in robot.sensors.parallel_right):
            print(clr, end="\r")
            print(" ! Obstacle parallel to the right side", end="\r")
            if robot.sensors.parallel_right[0] < robot.sensors.parallel_right[
                1]:
                lspeed, rspeed = 0, 0.5
            else:
                lspeed, rspeed = 1, 1
        else:
            lspeed, rspeed = 1, 1
    else:
        if robot.is_any_obstacle_right():
            print(clr, end="\r")
            print("! Obstacle right", end="\r")
            if robot.is_any_obstacle_left():
                print(clr, end="\r")
                print("! Obstacle left", end="\r")
                lspeed, rspeed = 10, -10
            else:
                lspeed, rspeed = 0.1, 1
        elif robot.is_any_obstacle_left():
            print(clr, end="\r")
            print("! Obstacle left", end="\r")
            if robot.is_any_obstacle_right():
                lspeed, rspeed = -10, 10
            else:
                lspeed, rspeed = 1, 0.1
        else:
            print(clr, end="\r")
            print(" >>> Going forward", end="\r")
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
        # sensors = Sensors()
        # robot = Pioneer(sensors)
        robot = PioneerMap(X0=-2, Y0=2, map_width=4, map_height=4,
                           client=clientID, grid_size=(50, 50), sonar=None)
        sensors = robot.sensors

        while vrep.simxGetConnectionId(clientID) != -1:
            # Perception
            sonar = getSonar(clientID, hRobot)
            x, y = getRobotPosition(clientID, hRobot)
            # print '### s', sonar

            # print('P: ', getRobotPosition(clientID, hRobot))
            # print('Th:', math.degrees(getRobotHeading(clientID, hRobot)))
            # print
            sensors.set_sonar(sonar)

            # Planning
            lspeed, rspeed = avoid(robot)

            # Action
            setSpeed(clientID, hRobot, lspeed, rspeed)
            robot.update_robot_position(x, y, sonar)
            time.sleep(0.1)

        print('### Finishing...')
        vrep.simxFinish(clientID)
        with open("grid.txt", "w") as file:
            for x, y in np.ndindex(robot.grid.shape):
                file.write(f" {robot.grid[x, y]} ")
                if (y + 1) == robot.grid.shape[1]:
                    file.write("\n")
        robot._print_task.close()
        # grid = np.ndarray((50, 50), dtype=np.float, buffer=sh_memory.buf)
        # for annotation in annotations_list:
        #     annotation.remove()
        # annotations_list[:] = list()
        nRows, nCols = 50, 50
        figure = plt.figure(figsize=(40, 40))
        axis = figure.add_subplot(111)
        image = axis.imshow(np.random.randint(0, 10, size=(nRows, nCols)),
                            cmap="gray_r")
        image.set_data(robot.grid)
        w, h = robot.grid.shape
        threshold = robot.grid.max() / 2.5
        for x, y in np.ndindex(robot.grid.shape):
            value = round(robot.grid[x, y], 2) if robot.grid[x, y] != 0 else 0
            annotation = axis.annotate(str(value),
                                       xy=(y, x),
                                       horizontalalignment="center",
                                       verticalalignment="center",
                                       color="white" if robot.grid[x, y] >
                                                        threshold else "black",
                                       size=4)
            # annotations_list.append(annotation)
        figure.canvas.draw_idle()
        # # plt.show()
        # plt.waitforbuttonpress()
        # plt.pause(.1)
        # fig, ax = plt.subplots()
        # ax.imshow(robot.grid, cmap=plt.cm.Greys, interpolation=None)
        # plt.show()
        del robot

    print('### Program ended')


# --------------------------------------------------------------------------

if __name__ == '__main__':
    main()
