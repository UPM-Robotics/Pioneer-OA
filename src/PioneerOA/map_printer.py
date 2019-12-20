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

matplotlib.use('TkAgg')  # MUST BE CALLED BEFORE IMPORTING plt
import matplotlib.pyplot as plt

# from scipy import misc
# from scipy import ndimage

from threading import current_thread
from time import time

from multiprocessing import Lock
from multiprocessing import Process
from multiprocessing import dummy as mp
from multiprocessing import shared_memory


def start_printing(robot_lock: Lock) -> Process:
    def _print_map(lock: Lock):
        # nRows, nCols = 150, 150
        shape = (1000, 1000)
        figure = plt.figure(figsize=(6, 6))
        axis = figure.add_subplot(111)
        image = axis.imshow(np.random.randint(0, 10, size=shape),
                            cmap="gray_r")
        # annotations_list = list()
        # annotation_lock = Lock()

        sh_memory = shared_memory.SharedMemory(name="pioneer-oa")
        annotations_grid = np.zeros(shape)

        # f = misc.face(gray=True)

        def annotate(index):
            value = round(grid[index], 2) if grid[index] != 0 else 0
            # color = "white" if grid[index] < threshold else "black"
            if value > 25 and annotations_grid[index] != grid[index]:
                axis.annotate(str(value),
                              xy=(index[1], index[0]),
                              horizontalalignment="center",
                              verticalalignment="center",
                              color="black",
                              size=4)
                annotations_grid[index] = grid[index]
            # with annotation_lock:
            #     annotations_list.append(annotation)

        nCores = os.cpu_count()
        if nCores is None:
            nCores = 4
        grid_pool = mp.Pool(nCores)

        plt.show(block=False)
        # figure.canvas.draw()
        # plt.pause(0.01)

        while True:
            with lock:
                grid = np.ndarray(shape,
                                  dtype=np.float,
                                  buffer=sh_memory.buf)
            # for annotation in annotations_list:
            #     annotation.remove()
            # annotations_list[:] = list()
            # image.set_data(ndimage.gaussian_filter(grid, sigma=2))
            image.set_data(grid)
            # w, h = grid.shape
            threshold = 5
            # startt = time()
            grid_pool.map(annotate, np.ndindex(grid.shape))
            # endt = time()
            # print(f"Rendering time: {(endt - startt):.3f}s")
            # figure.canvas.draw()
            # figure.canvas.flush_events()
            # figure.draw_artist(axis)
            # figure.canvas.update()
            # grid_pool.close()
            # grid_pool.join()
            # for x, y in np.ndindex(grid.shape):
            #     value = round(grid[x, y], 2) if grid[x, y] != 0 else 0
            #     annotation = axis.annotate(str(value),
            #                                xy=(y, x),
            #                                horizontalalignment="center",
            #                                verticalalignment="center",
            #                                color="white" if grid[x, y] <
            #                                                 threshold else "black",
            #                                size=4)
            #     annotations_list.append(annotation)
            figure.canvas.draw_idle()
            plt.pause(0.01)
            # figure.canvas.blit(axis.bbox)
            # figure.canvas.draw_idle()
            # plt.pause(0.01)

    _print_task = Process(target=_print_map, args=(robot_lock,))
    _print_task.start()
    return _print_task
