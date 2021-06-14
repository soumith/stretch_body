# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params

# TODO: Fails under PY3!
#stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import stretch_body.arm as arm
from stretch_body.transport import *

class TestTransport(unittest.TestCase):
    def test_bad_rpc(self):
        a = arm.Arm()
        a.startup()
        print('Running bad RPC test...')
        rpc=RPCRequest(id=999,callback=a.motor.rpc_status_reply)
        a.motor.transport.execute_rpc(rpc)
        rpc = RPCRequest(id=1, callback=a.motor.rpc_status_reply)
        a.motor.transport.execute_rpc(rpc)
        self.assertTrue(a.motor.transport.status['rpc_errors']==2)
        a.stop()
    def test_bad_rpc_async(self):
        a = arm.Arm()
        a.startup()
        print('Running bad async RPC test...')
        rpc=RPCRequest(id=999,callback=a.motor.rpc_status_reply)
        asyncio.get_event_loop().run_until_complete(a.motor.transport.execute_rpc_async(rpc))
        rpc = RPCRequest(id=1, callback=a.motor.rpc_status_reply)
        asyncio.get_event_loop().run_until_complete(a.motor.transport.execute_rpc_async(rpc))
        self.assertTrue(a.motor.transport.status['rpc_errors']==2)
        a.stop()
    def test_comm_load(self):
        a = arm.Arm()
        a.startup()
        print('Running comm load test...')
        a.motor.set_load_test()
        a.push_command()
        self.assertTrue(a.motor.transport.status['rpc_errors'] == 0)
        a.stop()
    def test_comm_load_async(self):
        a = arm.Arm()
        a.startup()
        print('Running async comm load test...')
        a.motor.set_load_test()
        asyncio.get_event_loop().run_until_complete(a.push_command_async())
        self.assertTrue(a.motor.transport.status['rpc_errors'] == 0)
        a.stop()



