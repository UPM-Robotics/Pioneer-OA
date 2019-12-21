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
import numpy as np

from mapping import PioneerMap

from multiprocessing import Lock
from multiprocessing import shared_memory


class Mapper(PioneerMap):
    def __init__(self,
                 X0: int,
                 Y0: int,
                 map_width: int,
                 map_height: int,
                 grid_size: tuple,
                 sonar: list = None,
                 k: float = 1,
                 min_cv: float = 0.5,
                 max_cv: float = 30,
                 max_read_distance: float = 1.0):
        super().__init__(X0, Y0, map_width, map_height, grid_size, sonar, k,
                         min_cv, max_cv, max_read_distance)
        self.lock = Lock()
        self.sh_memory = shared_memory.SharedMemory(name="pioneer-oa",
                                                    create=True,
                                                    size=self.grid.nbytes)
        self.shared_np = np.ndarray(self.grid.shape,
                                    dtype=np.float_,
                                    buffer=self.sh_memory.buf)
        self.shared_np[:] = self.grid[:]

    def update_robot_position(self,
                              robotX: float,
                              robotY: float,
                              sonar: list,
                              heading: float):
        super().update_robot_position(robotX, robotY, sonar, heading)
        with self.lock:
            self.shared_np[:] = self.grid[:]

    def __del__(self):
        try:
            self.sh_memory.close()
        finally:
            self.sh_memory.unlink()
