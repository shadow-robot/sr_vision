#!/usr/bin/env python

# Copyright 2019 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

"""
See README.md
"""

from base_calibration_test import BaseCalibrationTest


class TestTranslationAndRotation(BaseCalibrationTest):
    def test_camera_calibration(self):
        self.run_simple_test()


if __name__ == '__main__':
    import rostest

    rostest.rosrun(BaseCalibrationTest.get_package_name(),
                   'translation_and_rotation', TestTranslationAndRotation)
