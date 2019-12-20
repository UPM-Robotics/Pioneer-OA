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
import cython

import matplotlib as plt

import numpy as np

from libc.math cimport sin
from libc.math cimport cos
from libc.math cimport M_PI

# from cython.parallel import prange

from multiprocessing import Lock
from multiprocessing import Process
from multiprocessing import shared_memory


from sensors cimport Sensors
from pioneer cimport Pioneer
from PioneerSensor cimport PioneerSensor


# @cython.cclass
class PioneerMap(Pioneer):
    # cdef public int X0, Y0, w, h, mw, mh, max
    # cdef public np.ndarray grid
    # cdef public float k
    def __init__(self,
                 X0: int,
                 Y0: int,
                 map_width: int,
                 map_height: int,
                 grid_size: tuple,
                 sonar: list = None,
                 k: float = 1,
                 max_cv: int = 30,
                 max_read_distance: float = 1.0):
        super().__init__(Sensors(sonar))
        assert grid_size[0] > 0 and grid_size[1] > 0
        self.X0 = X0
        self.Y0 = Y0
        self.w = map_width
        self.h = map_height
        self.mw = grid_size[0]
        self.mh = grid_size[1]
        self.sensor = PioneerSensor(sonar)
        self.grid = np.zeros(shape=grid_size)
        self.k = k
        self.max = max_cv
        self.heading = 0
        self.max_read_distance = max_read_distance
        self.lock = Lock()
        self.sh_memory = shared_memory.SharedMemory(name="pioneer-oa",
                                                    create=True,
                                                    size=self.grid.nbytes)
        self.shared_np = np.ndarray(self.grid.shape,
                                    dtype=self.grid.dtype,
                                    buffer=self.sh_memory.buf)
        self.shared_np[:] = self.grid[:]
        # self.window = plt.figure(figsize=(40, 40))
        # self._print_task = Process(target=self._print_map, args=(self.lock,))
        # self._print_task.start()

    # @cython.locals(nRows=cython.int, nCols=cython.int,
    #                annotations_list=cython.list, grid=cython.list,
    #                threshold=cython.double)
    # @cython.boundscheck(False)  # turn off bounds-checking for entire function
    # @cython.wraparound(
    #     False)  # turn off negative index wrapping for entire function
    # @cython.boundscheck(False)
    # @cython.wraparound(False)
    # def _print_map(self, lock: Lock):
    #     nRows, nCols = 50, 50
    #     figure = plt.figure(figsize=(6, 6))
    #     axis = figure.add_subplot(111)
    #     image = axis.imshow(np.random.randint(0, 10, size=(nRows, nCols)),
    #                         cmap="gray_r")
    #     annotations_list = list()
    #
    #     sh_memory = shared_memory.SharedMemory(name="pioneer-oa")
    #
    #     while True:
    #         with lock:
    #             grid = np.ndarray((50, 50),
    #                               dtype=np.float,
    #                               buffer=sh_memory.buf)
    #         for annotation in annotations_list:
    #             annotation.remove()
    #         annotations_list[:] = list()
    #         image.set_data(grid)
    #         # w, h = grid.shape
    #         threshold = grid.max() / 1.5
    #         for x, y in np.ndindex(grid.shape):
    #             value = round(grid[x, y], 2) if grid[x, y] != 0 else 0
    #             annotation = axis.annotate(str(value),
    #                                        xy=(y, x),
    #                                        horizontalalignment="center",
    #                                        verticalalignment="center",
    #                                        color="white" if grid[x, y] >
    #                                                         threshold else "black",
    #                                        size=4)
    #             annotations_list.append(annotation)
    #         figure.canvas.draw_idle()
    #         plt.pause(.01)

    def _translate_to_matrix_position(self, x: float, y: float) -> tuple:
        return int((x - self.X0) * (self.mw / self.w)), \
               -int((y - self.Y0) * (self.mh / self.h))

    # @cython.cfunc
    def update_robot_position(self,
                              robotX: float,
                              robotY: float,
                              sonar: list,
                              heading: float):
        assert len(sonar) == 16
        for i in range(16):
            self.sensor[i].value = sonar[i]
            x = self.sensor[i].value * \
                cos(self.sensor[i].angle + heading) + robotX
            y = self.sensor[i].value * \
                sin(self.sensor[i].angle + heading) + robotY
            mX, mY = self._translate_to_matrix_position(x, y)
            if mX < self.grid.shape[0] and mY < self.grid.shape[1]:
                # cv = (1 - self.sensor[i].value) * self.k
                cv = self.k * (self.max_read_distance - self.sensor[i].value) \
                     / self.max_read_distance
                if cv > 0.5:
                    self.grid[mX, mY] += cv
        with self.lock:
            self.shared_np[:] = self.grid[:]

    def __del__(self):
        try:
            # self._print_task.terminate()
            # self._print_task.close()
            self.sh_memory.close()
        except:
            pass
