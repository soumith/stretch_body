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
        print('Running Encoder Flash R/W test...')

        self.assertTrue(a.motor.transport.status['rpc_errors']==2)
        a.stop()
