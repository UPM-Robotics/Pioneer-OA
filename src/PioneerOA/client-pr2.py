#!/home/javinator9889/.pyenv/shims/python
# /usr/bin/python3

print('### Script:', __file__)

import math
import sys

import numpy as np
import vrep

from mapper import Mapper
from map_printer import start_printing
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
            if robot.sensors.parallel_right[0] < robot.sensors.parallel_right[
                1]:
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
        # sensors = Sensors()
        # robot = Pioneer(sensors)
        robot = Mapper(X0=-2,
                       Y0=2,
                       map_width=4,
                       map_height=4,
                       grid_size=(500, 500),
                       initial_threshold=0.3,
                       max_read_distance=0.4,
                       ratio=1E-6)
        sensors = robot.sensors
        printer = start_printing(robot.lock)

        while vrep.simxGetConnectionId(clientID) != -1:
            # Perception
            sonar = getSonar(clientID, hRobot)
            x, y = getRobotPosition(clientID, hRobot)
            # print '### s', sonar

            # print('P: ', getRobotPosition(clientID, hRobot))
            # print('Th:', math.degrees(getRobotHeading(clientID, hRobot)))
            # print
            heading = getRobotHeading(clientID, hRobot)
            sensors.set_sonar(sonar)

            # Planning
            lspeed, rspeed = avoid(robot)

            # Action
            # print(f"{(x, y)}")
            setSpeed(clientID, hRobot, lspeed, rspeed)
            robot.update_robot_position(x, y,
                                        np.asarray(sonar, dtype=np.float_),
                                        heading)
            print(clr, end="\r")
            print(f"Threshold: {robot.threshold}", end="\r")
            # time.sleep(0.1)

        print('### Finishing...')
        vrep.simxFinish(clientID)
        with open("grid.txt", "w") as file:
            for x, y in np.ndindex(robot.grid.shape):
                file.write(f" {robot.grid[x, y]} ")
                if (y + 1) == robot.grid.shape[1]:
                    file.write("\n")

        printer.terminate()
        printer.join()
        printer.close()

        fig, ax = plt.subplots()
        ax.imshow(robot.grid, cmap=plt.cm.Greys, interpolation=None)
        plt.show()
        del robot

    print('### Program ended')


# --------------------------------------------------------------------------

if __name__ == '__main__':
    main()
