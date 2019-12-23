#                             sensors
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

# distutils: language=c++

import cython

import numpy as np
cimport numpy as np

cdef class Sensors:
    """
    Wrapper for managing the data input received by the robot sensors.
    As the range of each sonar is, at most, one meter, this wrapper converts each
    value to centimeters by multiplying by 100.

    In addition, this model is using only the first eight sensors as the back ones
    are not being used.

    The accessible params are:
     - sonar: a numpy array containing the sonar data (len == 8).
     - parallel_left: the sensors 1 and 16 for controlling the distance to the wall.
     - parallel_right: the sensors 8 and 9 for controlling the distance to the wall.
    """
    def __init__(self, list sonar = None):
        self.sonar = np.ones(8) * 100 if sonar is None else \
            np.asarray(sonar[:8]) * 100
        """
        numpy array containing the values for each sensor, in centimeters, starting from zero.
        """
        self.parallel_left = int(sonar[0] * 100) if sonar is not None else 1, \
                             int(sonar[15] * 100) if sonar is not None else 1
        """
        Tuple[int, int] containing the distance to the wall on the left, in centimeters.
        """
        self.parallel_right = int(sonar[7] * 100) if sonar is not None else 1, \
                              int(sonar[8] * 100) if sonar is not None else 1
        """
        Tuple[int, int] containing the distance to the wall on the right, in centimeters.
        """

    @cython.boundscheck(False)
    @cython.wraparound(False)
    cpdef set_sonar(self, list sonar):
        """
        Updates the sonar data. Input list contains the read data from sonar in meters.

        Parameters
        ----------
        sonar: list of float data
            measured in meters, whose length is 16.

        Raises
        ------
        AssertionError
            when the length of the list is distinct 16.
        """
        assert len(sonar) == 16
        self.sonar = np.asarray(sonar[:8]) * 100
        self.parallel_left = int(sonar[0] * 100), int(sonar[15] * 100)
        self.parallel_right = int(sonar[7] * 100), int(sonar[8] * 100)

    @property
    def front_sensors(self) -> np.float_:
        """
        Fast access for the front sensors of the robot.

        :return a numpy array from sensor number 3 to sensor number 5.
        """
        return self.sonar[2:6]

    @property
    def left_sensors(self) -> np.float_:
        """
        Fast access for the left sensors of the robot.

        :return a numpy array from sensor number 1 to sensor number 3.
        """
        return self.sonar[0:4]

    @property
    def right_sensors(self) -> np.float_:
        """
        Fast access for the front sensors of the robot.

        :return a numpy array from sensor number 5 to sensor number 7.
        """
        return self.sonar[5:8]

    def __getitem__(self, int key):
        assert isinstance(key, int)
        return self.sonar[key]

    def __reduce__(self):
        return Sensors, (self.sonar, self.parallel_left, self.parallel_right)
