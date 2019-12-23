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

from mapper import Mapper


class Wrapper:
    """
    Wrapper class that encapsulates a given Mapper class - it is useful for
    saving and restoring data from a created mapping.pyx object, as Cython is
    not providing object serialization for custom data types.
    """

    def __init__(self, mapper: Mapper):
        self.X0 = mapper.X0
        self.Y0 = mapper.Y0
        self.map_grid = (mapper.w, mapper.h)
        self.grid_size = (mapper.mw, mapper.mh)
        self.grid = np.asarray(mapper.grid, dtype=np.float_)
        self.k = mapper.k
        self.min = mapper.min
        self.max = mapper.max
        self.heading = mapper.heading
        self.max_read_distance = mapper.max_read_distance
        self.ratio = mapper.ratio
        self.threshold = mapper.threshold

    def restore(self) -> Mapper:
        """
        Restores the class into a Mapper class, setting the old params into
        the new class instance.
        :return: the recovered mapper class.
        """
        mapper = Mapper(X0=self.X0,
                        Y0=self.Y0,
                        map_width=self.map_grid[0],
                        map_height=self.map_grid[1],
                        grid_size=self.grid_size,
                        k=self.k,
                        max_read_distance=self.max_read_distance,
                        ratio=self.ratio)
        mapper.grid = self.grid
        mapper.min = self.min
        mapper.max = self.max
        mapper.heading = self.heading
        mapper.threshold = self.threshold

        return mapper
