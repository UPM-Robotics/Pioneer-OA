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
import cython

cimport numpy as np

import matplotlib.pyplot as plt

from cython.parallel import prange

from multiprocessing import Lock
from multiprocessing import shared_memory

def print_map(lock: Lock):
    nRows, nCols = 100, 100
    figure = plt.figure(figsize=(6, 6))
    axis = figure.add_subplot(111)
    image = axis.imshow(np.random.randint(0, 10, size=(nRows, nCols)),
                        cmap="gray_r")
    annotations_list = list()

    sh_memory = shared_memory.SharedMemory(name="pioneer-oa")

    while 1:
        with lock:
            grid = np.ndarray((100, 100),
                              dtype=np.float,
                              buffer=sh_memory.buf)
            for annotation in annotations_list:
                annotation.remove()
            annotations_list[:] = list()
            # image.set_data(ndimage.gaussian_filter(grid, sigma=2))
            image.set_data(grid)
            # w, h = grid.shape
            threshold = 5
            xmax = grid.shape[0]
            ymax = grid.shape[1]
            for x in prange(xmax, nogil=True):
                for y in prange(ymax, nogil=True):
                    value = round(grid[x, y], 2) if grid[x, y] != 0 else 0
                    annotation = axis.annotate(str(value),
                                               xy=(y, x),
                                               horizontalalignment="center",
                                               verticalalignment="center",
                                               color="white" if grid[x, y] <
                                                                threshold else "black",
                                               size=4)
                    # annotations_list.append(annotation)
            figure.canvas.draw_idle()
            plt.pause(0.01)
