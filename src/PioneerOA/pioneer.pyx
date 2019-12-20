#                             robot_control
#                  Copyright (C) 2019 - Javinator9889
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#      the Free Software Foundation, either version 3 of the License, or
#                   (at your option) any later version.
#
#       This program is distributed in the hope that it will be useful,
#       but WITHOUT ANY WARRANTY; without even the implied warranty of
#        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#               GNU General Public License for more details.
#
#     You should have received a copy of the GNU General Public License
#    along with this program. If not, see <http://www.gnu.org/licenses/>.
import cython

import numpy as np

from libc.math cimport sin
from libc.math cimport cos
from libc.math cimport M_PI

# from math import sin
# from math import cos
# from math import radians

from typing import Dict
from typing import List
from typing import Tuple

# from cython.parallel import prange

from sensors import Sensors

cpdef float radians(float degrees):
    return (degrees * M_PI) / 180.0

# @cython.cclass
cdef class Pioneer:
    """
    Direct access the robot information wrapping the different values
    contained in sensors.

    With this class, it is possible to know the exact situation of the
    robot and actuate in base on it.

    The accessible parameter is:
     - sensors: an instance of Sensors class wrapping the different sonar values.
    """

    def __init__(self, sensors: Sensors):
        self.sensors = sensors

    cpdef cython.float nearest_obstacle_at(self, str orientation):
        """
        With the given orientation, finds the nearest obstacle to it.

        Parameters
        ----------
        orientation : str
            The orientation in which the robot must look for
            obstacles - possible values are: `{right, left, front}`.
        Returns
        -------
        out : float
            The nearest obstacle to the given orientation. If the orientation
            is not valid, then the result is -1.
        """
        if orientation == "left":
            return self.sensors.left_sensors.min()
        elif orientation == "right":
            return self.sensors.right_sensors.min()
        elif orientation == "front":
            return self.sensors.front_sensors.min()
        else:
            return -1

    # @cython.cfunc
    cpdef cython.int is_any_obstacle_front(self):
        """
        Checks if there is any obstacle in front of the robot (sensors 3 to 5).

        Returns
        -------
        out : bool
            True if some of the sensors is less or equal to the minimum distance.
            If all of the sensors do not detect any obstacle, then returns False.
        """
        for i in range(2, 6):
            min_dist, dist = self.distance_in_xaxis(i)
            if dist <= min_dist:
                return 1
        return 0

    # @cython.cfunc
    cpdef cython.int is_any_obstacle_left(self):
        """
        Checks if there is any obstacle next to the robot on the left (sensors 2 to 4).

        Returns
        -------
        out : bool
            True if some of the sensors is less or equal to the minimum distance.
            If all of the sensors do not detect any obstacle, then returns False.
        """
        for i in range(1, 5):
            min_dist, dist = self.distance_in_xaxis(i)
            if dist <= min_dist:
                return 1
        return 0

    # @cython.cfunc
    cpdef cython.int is_any_obstacle_right(self):
        """
        Checks if there is any obstacle next to the robot on the right (sensors 6 to 8).

        Returns
        -------
        out : bool
            True if some of the sensors is less or equal to the minimum distance.
            If all of the sensors do not detect any obstacle, then returns False.
        """
        for i in range(5, 8):
            min_dist, dist = self.distance_in_xaxis(i)
            if dist <= min_dist:
                return 1
        return 0

    # @cython.cfunc
    cpdef tuple distance_in_xaxis(self, int sensor):
        """
        Calculates the distance in the x-axis by using the cosine of the angle multiplied
        by the measured distance.

        Parameters
        ----------
        sensor : int
            the position of the sensor (from 0 to 7) from which the distance must be
            measured.

        Returns
        -------
        out : Tuple[float, float]
            the minimum distance, for comparing, and the projection of the distance in
            the x-axis.
        """
        min_distance, angle, dist = self[sensor]
        return min_distance, dist * cos(angle)

    def __getitem__(self, key) -> tuple:
        assert isinstance(key, int)
        dist = {
            7: 0.5,
            6: 2,
            5: 3,
            4: 8,
            3: 8,
            2: 3,
            1: 2,
            0: 0.5
        }.get(key)
        angle = {
            7: radians(0),  # 90º - 90º
            6: radians(40),  # 90º - 50º
            5: radians(60),  # 90º - 30º
            4: radians(80),  # 90º - 10º
            3: radians(80),  # 90º - 10º
            2: radians(60),  # 90º - 30º
            1: radians(40),  # 90º - 50º
            0: radians(0)  # 90º - 90º
        }.get(key)
        return dist, angle, self.sensors[key]
