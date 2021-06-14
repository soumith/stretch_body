# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params

# TODO: Fails under PY3!
#stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import stretch_body.arm as arm
from stretch_body.transport import *

class TestStepper(unittest.TestCase):
    def test_encoder_flash_rw(self):
        a = arm.Arm()
        a.startup()
        # print('------------------------------------')
        # print('Running Encoder Flash R/W test...')
        # print('Reading calibration data from YAML...')
        # data = a.motor.read_encoder_calibration_from_YAML()
        # print('Writing calibration data to flash...')
        # a.motor.write_encoder_calibration_to_flash(data)
        # print('Successful write of FLASH. Resetting board now.')
        # a.motor.board_reset()
        # a.motor.push_command()
        print('------------------------------------')
        data2 = a.motor.read_encoder_calibration_from_flash()
        print('Read data of len', len(data))
        # print('Comparing data to YAML')
        # failures=0
        # for i in range(len(data)):
        #     if data[i]!=data2[i]:
        #         failures=failures+1
        # self.assertTrue(failures==0)
        a.stop()
