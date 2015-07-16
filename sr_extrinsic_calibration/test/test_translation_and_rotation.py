#!/usr/bin/env python
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
