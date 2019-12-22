#                             src
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
from libc.math cimport sin
from libc.math cimport cos


cdef class Sensor:
    def __init__(self, double angle, double value = 0.0):
        self.angle = angle
        self.value = value

    cpdef double x(self, double robotX, double heading):
        return robotX + 0.2275 * cos(self.angle + heading)

    cpdef double y(self, double robotY, double heading):
        return robotY + 0.2275 * sin(self.angle + heading)

