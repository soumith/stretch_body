# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params

# TODO: Fails under PY3!
#stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import stretch_body.stepper as stepper
import stretch_body.pimu as pimu
import stretch_body.wacc as wacc
import stretch_body.robot as robot
import time
import asyncio
from concurrent.futures import ThreadPoolExecutor


def create_devices():
    s1 = stepper.Stepper('/dev/hello-motor-lift')
    s1.startup()
    s2 = stepper.Stepper('/dev/hello-motor-arm')
    s2.startup()
    s3 = stepper.Stepper('/dev/hello-motor-left-wheel')
    s3.startup()
    s4 = stepper.Stepper('/dev/hello-motor-right-wheel')
    s4.startup()
    w = wacc.Wacc()
    w.startup()
    p = pimu.Pimu()
    p.startup()
    return p,w,s1,s2,s3,s4

def stop_devices(p,w,s1,s2,s3,s4):
    p.stop()
    w.stop()
    s1.stop()
    s2.stop()
    s3.stop()
    s4.stop()

class TestAsyncioDevice(unittest.TestCase):
    def test_concurrent_access(self):
        """
        Verify zero comms errors when access the non-async pull_status
        while the async robot pull_status thread runs
        """
        print('Testing Concurrent Robot Access')
        r = robot.Robot()
        r.startup()

        def device_n(n):
            print('N',n)
            for i in range(20):
                if n==1:
                    r.pimu.pull_status()
                if n==2:
                    r.wacc.pull_status()
                if n==3:
                    r.base.pull_status()
                if n==4:
                    r.lift.pull_status()
                if n==5:
                    r.arm.pull_status()
                time.sleep(0.01)

        ns = [1,2,3,4,5]
        with ThreadPoolExecutor(max_workers = 2) as executor:
            results = executor.map(device_n, ns)
        self.assertTrue(1) #TODO, read comm errors from device

        r.stop()

    def test_push_command_rate(self):
        p, w, s1, s2, s3, s4 = create_devices()
        loop = asyncio.get_event_loop()
        ts = time.time()
        n = 100
        print('Running Async Push Command test...')
        for i in range(n):
            p.runstop_event_trigger()
            w.set_D2(1)
            s1.enable_safety()
            s2.enable_safety()
            s3.enable_safety()
            s4.enable_safety()
            coro = asyncio.gather(p.push_command_async(), w.push_command_async(),
                                  s1.push_command_async(), s2.push_command_async(),
                                  s3.push_command_async(), s4.push_command_async())
            loop.run_until_complete(coro)

        avg_ms = ((time.time() - ts) / n) * 1000.0
        rate = 1000 / avg_ms
        print('Average push_command time of %.2f ms' % avg_ms)
        print('Rate of %.2f Hz' % rate)
        self.assertTrue(rate > 50.0 and rate < 70.0)
        stop_devices(p, w, s1, s2, s3, s4)

    def test_pull_status_rate(self):
        p, w, s1, s2, s3, s4=create_devices()
        loop = asyncio.get_event_loop()
        ts = time.time()
        n = 100
        print('Running Async Pull Status test...')
        for i in range(n):
            coro = asyncio.gather(p.pull_status_async(),w.pull_status_async(),
                                  s1.pull_status_async(), s2.pull_status_async(),
                                  s3.pull_status_async(), s4.pull_status_async())
            loop.run_until_complete(coro)

        avg_ms = ((time.time() - ts) / n) * 1000.0
        rate=1000/avg_ms
        print('Average pull_status time of %.2f ms' % avg_ms)
        print('Rate of %.2f Hz' % rate)
        self.assertTrue(rate>30.0 and rate<50.0)
        stop_devices(p,w,s1,s2,s3,s4)


