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
import numpy as np

import matplotlib.pyplot as plt

from math import sin
from math import cos

from typing import List
from typing import Tuple

from multiprocessing import Lock
from multiprocessing import Process
from multiprocessing import shared_memory

from .pioneer import Pioneer
from .pioneer import PioneerSensor

from .sensors import Sensors


class PioneerMap(Pioneer):
    def __init__(self,
                 X0: int,
                 Y0: int,
                 map_width: int,
                 map_height: int,
                 client,
                 grid_size: Tuple[int, int],
                 sonar: List[float] = None,
                 k: float = 1,
                 max_cv: int = 30):
        super().__init__(Sensors(sonar))
        assert grid_size[0] > 0 and grid_size[1] > 0
        self.X0 = X0
        self.Y0 = Y0
        self.w = map_width
        self.h = map_height
        self.mw = grid_size[0]
        self.mh = grid_size[1]
        self.client = client
        self.sensor = PioneerSensor(sonar)
        self.grid = np.zeros(shape=grid_size)
        self.k = k
        self.max = max_cv
        self.finished = False
        self.lock = Lock()
        self.sh_memory = shared_memory.SharedMemory(name="pioneer-oa",
                                                    create=True,
                                                    size=self.grid.nbytes)
        self.shared_np = np.ndarray(self.grid.shape,
                                    dtype=self.grid.dtype,
                                    buffer=self.sh_memory.buf)
        self.shared_np[:] = self.grid[:]
        # self.window = plt.figure(figsize=(40, 40))
        self._print_task = Process(target=self._print_map, args=(self.lock,))
        self._print_task.start()

    def _print_map(self, lock):
        nRows, nCols = 50, 50
        figure = plt.figure(figsize=(6, 6))
        axis = figure.add_subplot(111)
        image = axis.imshow(np.random.randint(0, 10, size=(nRows, nCols)),
                            cmap="gray_r")
        annotations_list = list()

        sh_memory = shared_memory.SharedMemory(name="pioneer-oa")

        while True:
            with lock:
                grid = np.ndarray((50, 50),
                                  dtype=np.float,
                                  buffer=sh_memory.buf)
            for annotation in annotations_list:
                annotation.remove()
            annotations_list[:] = list()
            image.set_data(grid)
            # w, h = grid.shape
            threshold = grid.max() / 1.5
            for x, y in np.ndindex(grid.shape):
                value = round(grid[x, y], 2) if grid[x, y] != 0 else 0
                annotation = axis.annotate(str(value),
                                           xy=(y, x),
                                           horizontalalignment="center",
                                           verticalalignment="center",
                                           color="white" if grid[x, y] >
                                                            threshold else "black",
                                           size=4)
                annotations_list.append(annotation)
            figure.canvas.draw_idle()
            plt.pause(.01)

    def _translate_to_matrix_position(self, x, y) -> Tuple[int, int]:
        return int((x - self.X0) * (self.mw / self.w)), \
               -int((y - self.Y0) * (self.mh / self.h))

    def update_robot_position(self,
                              robotX: float,
                              robotY: float,
                              sonar: List[float]):
        assert len(sonar) == 16
        for i in range(16):
            self.sensor[i].value = sonar[i]
            x = self.sensor[i].value * cos(self.sensor[i].angle) + robotX
            y = self.sensor[i].value * sin(self.sensor[i].angle) + robotY
            mX, mY = self._translate_to_matrix_position(x, y)
            if mX < self.grid.shape[0] and mY < self.grid.shape[1]:
                cv = (1 - self.sensor[i].value) * self.k
                self.grid[mX, mY] += cv
        with self.lock:
            self.shared_np[:] = self.grid[:]

    def __del__(self):
        try:
            self._print_task.terminate()
            self._print_task.close()
            self.sh_memory.close()
        except:
            pass
