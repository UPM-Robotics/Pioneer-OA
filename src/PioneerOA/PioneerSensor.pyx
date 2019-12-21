#                             robot_control
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

# distutils: language=c++
from sensor cimport Sensor
from pioneer cimport radians

cdef class PioneerSensor:
    def __init__(self, list sonar = None):
        angles = {
            0: radians(90),
            1: radians(50),
            2: radians(30),
            3: radians(10),
            4: radians(-10),
            5: radians(-30),
            6: radians(-50),
            7: radians(-90),
            8: radians(-90),
            9: radians(-130),
            10: radians(-150),
            11: radians(-170),
            12: radians(170),
            13: radians(150),
            14: radians(130),
            15: radians(90)
        }
        if sonar is None:
            sonar = [1] * 16
        assert len(sonar) == 16
        self.sonar = [Sensor(angle, value) for angle, value in zip(
            angles.values(), sonar)]
        self.parallel_left = [self.sonar[15], self.sonar[0]]
        self.parallel_right = [self.sonar[7], self.sonar[8]]

    def __getitem__(self, item) -> Sensor:
        assert isinstance(item, int)
        return self.sonar[item]
