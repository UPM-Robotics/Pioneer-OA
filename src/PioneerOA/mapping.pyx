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

# cython: infer_types=True
# distutils: language=c++
import cython

import numpy as np
cimport numpy as np

from libc.math cimport sin
from libc.math cimport cos

from sensors cimport Sensors
from pioneer cimport Pioneer
from PioneerSensor cimport PioneerSensor


cdef class PioneerMap(Pioneer):
    def __init__(self,
                 double X0,
                 double Y0,
                 double map_width,
                 double map_height,
                 tuple grid_size,
                 list sonar = None,
                 double initial_threshold = 1.0,
                 double k = 1,
                 double min_cv = 0.5,
                 double max_cv = 30.0,
                 double max_read_distance = 1.0,
                 double ratio = 1.5):
        super().__init__(Sensors(sonar))
        assert grid_size[0] > 0 and grid_size[1] > 0
        self.X0 = X0
        self.Y0 = Y0
        self.w = map_width
        self.h = map_height
        self.mw = grid_size[0]
        self.mh = grid_size[1]
        self.sensor = PioneerSensor(sonar)
        self.grid = np.zeros(shape=grid_size, dtype=np.float_)
        self.k = k
        self.min = min_cv
        self.max = max_cv
        self.heading = 0
        self.max_read_distance = max_read_distance
        self.ratio = ratio
        self.threshold = initial_threshold

    cpdef tuple translate_to_matrix_position(self, double x, double y):
        return int((x - self.X0) * (self.mw / self.w)), \
               -int((y - self.Y0) * (self.mh / self.h))

    @cython.boundscheck(False)
    @cython.wraparound(False)
    @cython.locals(x=cython.double, y=cython.double, mX=cython.int,
                   mY=cython.int, cv=cython.double)
    cpdef update_robot_position(self,
                                double robotX,
                                double robotY,
                                double[:] sonar,
                                double heading):
        assert len(sonar) == 16
        for i in range(16):
            self.sensor[i].value = sonar[i]
            x = self.sensor[i].value * \
                cos(self.sensor[i].angle + heading) + robotX
            y = self.sensor[i].value * \
                sin(self.sensor[i].angle + heading) + robotY
            mX, mY = self.translate_to_matrix_position(x, y)
            if mX < self.grid.shape[0] and mY < self.grid.shape[1]:
                if self.sensor[i].value > self.max_read_distance:
                    continue
                cv = self.k * (self.max_read_distance -
                               self.sensor[i].value) / \
                     self.max_read_distance
                if cv >= self.threshold:
                    self.grid[mX, mY] += cv
                    self.threshold *= (1 + 0.1 * self.ratio)
                elif cv != 0:
                    self.threshold *= (1 - 0.1 * self.ratio)

    def __reduce__(self):
        return PioneerMap, (self.X0, self.Y0, self.w, self.h, self.mw, self.mh,
                            self.sensor, np.asarray(self.grid, dtype=np.float_),
                            self.k, self.min, self.max, self.heading,
                            self.max_read_distance, self.ratio,
                            self.threshold)
