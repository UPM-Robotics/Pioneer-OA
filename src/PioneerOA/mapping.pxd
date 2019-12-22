#                             PioneerOA
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

from pioneer cimport Pioneer
from PioneerSensor cimport PioneerSensor


cdef class PioneerMap(Pioneer):
    cdef public int X0, Y0, w, h, mw, mh
    cdef public double[:, :] grid
    cdef public double k, heading, max_read_distance, min, max, \
        threshold_divider
    cdef public PioneerSensor sensor

    cpdef tuple translate_to_matrix_position(self, double x, double y)

    @cython.locals(x=cython.double, y=cython.double, mX=cython.int,
                   mY=cython.int,
                   cv=cython.double)
    cpdef update_robot_position(self, double robotX, double robotY,
                                double[:] sonar, double heading)
