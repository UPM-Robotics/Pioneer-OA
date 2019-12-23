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

# distutils: language=c++
cimport cython

from sensors cimport Sensors

cpdef inline float radians(float degrees)

cdef class Pioneer:
    cdef public Sensors sensors

    cpdef double nearest_obstacle_at(self, str orientation)

    cpdef int is_any_obstacle_front(self)

    cpdef int is_any_obstacle_left(self)

    cpdef int is_any_obstacle_right(self)

    cpdef tuple distance_in_xaxis(self, int sensor)
