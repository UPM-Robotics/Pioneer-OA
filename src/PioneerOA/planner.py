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

from math import sqrt

from mapping import PioneerMap


class Planner:
    """
    Class that uses a given grid for planning a path to the robot.
    """

    def __init__(self, grid_map: PioneerMap):
        self.map = grid_map
        self.potential_map = grid_map.grid.copy()
        self.visited_map = np.zeros(grid_map.grid.shape, dtype=bool)

    def calculate_path(self, origin: tuple, target: tuple) -> list:
        """
        Calculates a path for the robot by using the origin and target points.
        :param origin: the origin point.
        :param target: the target point.
        :return: a list with the movements.
        """

        def is_valid_cell(i, j):
            return 0 <= i < self.potential_map.shape[0] and 0 <= j < \
                   self.potential_map.shape[1] and not self.visited_map[i, j]

        i0, j0 = self.map.translate_to_matrix_position(x=origin[0],
                                                       y=origin[1])
        it, jt = self.map.translate_to_matrix_position(x=target[0],
                                                       y=target[1])

        if self.map.grid[i0, j0] != 0 or self.map.grid[it, jt] != 0:
            return []

        for i, j in np.ndindex(self.map.grid.shape):
            if self.map.grid[i, j] != 0:
                self.potential_map[i, j] = -1
            else:
                if not ((i, j) == (it, jt)):
                    self.potential_map[i, j] = 1 / sqrt((it - i) ** 2
                                                        + (jt - j) ** 2)
                else:
                    self.potential_map[i, j] = float("inf")

        path = [(i0, j0)]
        self.visited_map[i0, j0] = True
        i, j = i0, j0
        while not ((i, j) == (it, jt)):
            self.visited_map[i, j] = True
            if (i, j) not in path:
                path.append((i, j))
            potentials = {
                "i + 1": 0,
                "i - 1": 0,
                "j + 1": 0,
                "j - 1": 0
            }
            if is_valid_cell(i + 1, j):
                potentials["i + 1"] = \
                    self.potential_map[i + 1, j]
            if is_valid_cell(i - 1, j):
                potentials["i - 1"] = \
                    self.potential_map[i - 1, j]
            if is_valid_cell(i, j + 1):
                potentials["j + 1"] = \
                    self.potential_map[i, j + 1]
            if is_valid_cell(i, j - 1):
                potentials["j - 1"] = \
                    self.potential_map[i, j - 1]

            movement = max(zip(potentials.values(), potentials.keys()))[1]
            if movement == "i + 1" and potentials[movement] >= 0:
                i += 1
            elif movement == "i - 1" and potentials[movement] >= 0:
                i -= 1
            elif movement == "j + 1" and potentials[movement] >= 0:
                j += 1
            elif movement == "j - 1" and potentials[movement] >= 0:
                j -= 1
            else:
                break

        path.append((it, jt))

        return path
