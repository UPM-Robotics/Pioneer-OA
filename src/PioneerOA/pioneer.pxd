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
cimport cython

# from libcpp cimport bool
# from cpython cimport bool

cimport numpy as np

cdef float radians(float degrees)

cdef class Pioneer:
    # def __init__(self, sensors: Sensors)

    cpdef cython.float nearest_obstacle_at(self, str orientation)

    cpdef cython.int is_any_obstacle_front(self)

    cpdef cython.int is_any_obstacle_left(self)

    cpdef cython.int is_any_obstacle_right(self)

    cpdef tuple distance_in_xaxis(self, int sensor)

    # def tuple __getitem__(self, int key)
