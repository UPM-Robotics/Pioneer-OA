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

from multiprocessing import Lock
from multiprocessing import Process
from multiprocessing import shared_memory


def start_printing(robot_lock: Lock) -> Process:
    def _print_map(lock: Lock):
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
            plt.pause(0.01)

    _print_task = Process(target=_print_map, args=(robot_lock,))
    _print_task.start()
    return _print_task
