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
import os

import numpy as np

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

from multiprocessing import Lock
from multiprocessing import Process
from multiprocessing import dummy as mp
from multiprocessing import shared_memory


def start_printing(robot_lock: Lock) -> Process:
    def _print_map(lock: Lock):
        sh_args = shared_memory.ShareableList(name="shared_args")
        shape = (sh_args[0], sh_args[1])
        figure = plt.figure(figsize=(6, 6))
        axis = figure.add_subplot(111)
        image = axis.imshow(np.random.randint(0, 10, size=shape),
                            cmap="gray_r")

        sh_memory = shared_memory.SharedMemory(name="pioneer-oa")
        annotations_grid = np.zeros(shape)

        def annotate(index):
            if threshold < grid[index] != annotations_grid[index]:
                axis.annotate('■',
                              xy=(index[1], index[0]),
                              horizontalalignment="center",
                              verticalalignment="center",
                              color="black",
                              size=4)
                annotations_grid[index] = grid[index]

        nCores = os.cpu_count()
        if nCores is None:
            nCores = 1
        grid_pool = mp.Pool(nCores)

        plt.show(block=False)

        while True:
            with lock:
                grid = np.ndarray(shape,
                                  dtype=np.float_,
                                  buffer=sh_memory.buf)
                threshold = sh_args[2]
            image.set_data(grid)
            grid_pool.map(annotate, np.ndindex(grid.shape))
            figure.canvas.draw_idle()
            plt.pause(0.01)

    _print_task = Process(target=_print_map, args=(robot_lock,))
    _print_task.start()
    return _print_task
