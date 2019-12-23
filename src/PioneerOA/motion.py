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
from math import sin
from math import cos

from mapping import PioneerMap


class Motion:
    def __init__(self, robot: PioneerMap, movements: list):
        self.robot = robot
        self.movements = movements

    def move(self, get_position, get_heading, set_speed, **kwargs):
        for movement in self.movements:
            x, y = get_position(kwargs["client"], kwargs["robot"])
            heading = get_heading(kwargs["client"], kwargs["robot"])
            i, j = self.robot.translate_to_matrix_position(x, y)
            diffi, diffj = movement[0] - i, movement[1] - j
            if diffi != 0:
                if abs(round(cos(heading), 2)) != 0:
                    if cos(heading) < 0:
                        if diffi > 0:
                            lspeed, rspeed = 0.5, 0
                            fval = 1
                        else:
                            lspeed, rspeed = 0, 0.5
                            fval = -1
                    else:
                        if diffi < 0:
                            lspeed, rspeed = 0, 0.5
                            fval = 1
                        else:
                            lspeed, rspeed = 0.5, 0
                            fval = -1
                    while round(sin(get_heading(kwargs["client"],
                                                kwargs["robot"])), 2) != fval:
                        set_speed(kwargs["client"], kwargs["robot"],
                                  lspeed, rspeed)
            elif diffj != 0:
                if abs(round(sin(heading), 2)) != 0:
                    if sin(heading) < 0:
                        if diffj > 0:
                            lspeed, rspeed = 0.5, 0
                            fval = 1
                        else:
                            lspeed, rspeed = 0, 0.5
                            fval = -1
                    else:
                        if diffj < 0:
                            lspeed, rspeed = 0, 0.5
                            fval = 1
                        else:
                            lspeed, rspeed = 0.5, 0
                            fval = -1
                    while round(cos(get_heading(kwargs["client"],
                                                kwargs["robot"])), 2) != fval:
                        set_speed(kwargs["client"], kwargs["robot"],
                                  lspeed, rspeed)
            while get_position(kwargs["client"], kwargs["robot"]) != \
                    movement:
                set_speed(kwargs["client"], kwargs["robot"], 1, 1)
