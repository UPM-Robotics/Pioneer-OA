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
    """
    Python-wide class for accessing the C interface and offering the
    necessary synchronization mechanisms in order to interact with other
    processes.
    This class creates a C instance of the PioneerMap class and a shared
    memory location for communicating with the map printer class.
    """

    def __init__(self,
                 X0: float,
                 Y0: float,
                 map_width: float,
                 map_height: float,
                 grid_size: tuple,
                 sonar: list = None,
                 initial_threshold: float = 1.0,
                 k: float = 1.0,
                 min_cv: float = 0.5,
                 max_cv: float = 30,
                 max_read_distance: float = 1.0,
                 ratio: float = 1.5):
        print(initial_threshold)
        super().__init__(X0, Y0, map_width, map_height, grid_size, sonar,
                         initial_threshold, k, min_cv, max_cv,
                         max_read_distance, ratio)
        self.lock = Lock()
        self.sh_memory = shared_memory.SharedMemory(name="pioneer-oa",
                                                    create=True,
                                                    size=self.grid.nbytes)
        self.shared_np = np.ndarray(self.grid.shape,
                                    dtype=np.float_,
                                    buffer=self.sh_memory.buf)
        self.shared_args = shared_memory.ShareableList([self.mw, self.mh,
                                                        self.threshold],
                                                       name="shared_args")
        self.shared_np[:] = self.grid[:]

    def update_robot_position(self,
                              robotX: float,
                              robotY: float,
                              sonar: list,
                              heading: float):
        """
        Updates the robot position and sets the shared data required
        information.
        :param robotX: double robot X position.
        :param robotY: double robot Y position.
        :param sonar: the list of the sensor data.
        :param heading: the robot heading (orientation).
        :return:
        """
        super().update_robot_position(robotX, robotY, sonar, heading)
        with self.lock:
            self.shared_np[:] = self.grid[:]
            self.shared_args[2] = self.threshold

    def __del__(self):
        """
        Safely finishes the class instance, removing the unnecessary and
        opened shared memory locations.
        :return:
        """
        try:
            self.sh_memory.close()
            self.shared_args.shm.close()
        finally:
            self.sh_memory.unlink()
            self.shared_args.shm.unlink()
            del self.shared_args

    def __reduce__(self):
        _, values = super().__reduce__()
        return Mapper, values
