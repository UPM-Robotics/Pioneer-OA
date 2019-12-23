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

from distutils.core import setup
from Cython.Build import cythonize

sourcefiles = ["mapping.pyx", "sensors.pyx", "pioneer.pyx", "sensor.pyx",
               "PioneerSensor.pyx"]
threads = os.cpu_count()
if threads is None:
    threads = 1

print("---------------------------------")
print(f"Compiling with {threads} threads")
print("---------------------------------")

setup(
    ext_modules=cythonize(sourcefiles,
                          nthreads=threads,
                          compiler_directives={"language_level": "3",
                                               "infer_types": True,
                                               "optimize.use_switch": True},
                          include_path=[np.get_include()],
                          annotate=True)
)
