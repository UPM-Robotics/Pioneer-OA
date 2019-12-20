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
cimport cython

from libcpp cimport bool

cimport numpy as np

cdef class Sensors:
    cpdef __init__(self, list sonar=None)

    cpdef set_sonar(self, list sonar)

    @property
    cpdef np.ndarray front_sensors(self)

    @property
    cpdef np.ndarray left_sensors(self)

    @property
    cpdef np.ndarray right_sensors(self)

    cpdef np.float32 __getitem__(self, int key)
