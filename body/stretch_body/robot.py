from __future__ import print_function
import threading
import time
import signal
import os
import importlib
import asyncio

from stretch_body.device import Device
import stretch_body.base as base
import stretch_body.arm as arm
import stretch_body.lift as lift
import stretch_body.pimu as pimu
import stretch_body.head as head
import stretch_body.wacc as wacc
import logging
import stretch_body.hello_utils as hello_utils

from serial import SerialException

from stretch_body.robot_monitor import RobotMonitor
from stretch_body.robot_collision import RobotCollision


# #############################################################
class DXLStatusThread(threading.Thread):
    """
    Do  pull_status data from the dxl devices
    """
    def __init__(self,robot):
        threading.Thread.__init__(self)
        self.robot=robot
        self.update_rate_hz = 15.0  #Hz
        self.stats = hello_utils.LoopStats(loop_name='DXLStatusThread',target_loop_rate=self.update_rate_hz)
        self.shutdown_flag = threading.Event()
        self.running=False
        self.logger = logging.getLogger()
    def run(self):
        while not self.shutdown_flag.is_set():
            self.stats.mark_loop_start()
            self.running = True
            ts = time.time()
            self.robot.end_of_arm.pull_status()
            self.robot.head.pull_status()
            self.stats.mark_loop_end()
            if not self.shutdown_flag.is_set():
                time.sleep(self.stats.get_loop_sleep_time())
        self.logger.debug('Shutting down DXLStatusThread')

class NonDXLStatusThread(threading.Thread):
    """
    Do Asyncio pull_status data from the non-dxl devices
    """
    def __init__(self,robot):
        threading.Thread.__init__(self)
        self.robot=robot
        self.update_rate_hz = 20.0  #Hz
        self.shutdown_flag = threading.Event()
        self.stats = hello_utils.LoopStats(loop_name='NonDXLStatusThread',target_loop_rate=self.update_rate_hz)
        self.running = False
        self.logger = logging.getLogger()
    def run(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        while not self.shutdown_flag.is_set():
            self.stats.mark_loop_start()
            self.running = True
            ts = time.time()
            loop.run_until_complete(asyncio.gather(
                self.robot.wacc.pull_status_async(),
                self.robot.base.pull_status_async(),
                self.robot.lift.pull_status_async(),
                self.robot.arm.pull_status_async(),
                self.robot.pimu.pull_status_async()))
            self.stats.mark_loop_end()
            if not self.shutdown_flag.is_set():
                time.sleep(self.stats.get_loop_sleep_time())
        print('Shutting down NonDXLStatusThread')

class RobotSafetyThread(threading.Thread):
    """
    Do  pull_status data from the dxl devices
    """
    def __init__(self,robot):
        threading.Thread.__init__(self)
        self.robot=robot
        self.update_rate_hz = 10.0  #Hz
        self.sentry_downrate_int = 1  # Step the sentry at every Nth iteration
        self.collision_downrate_int = 2  # Step the sentry at every Nth iteration
        self.monitor_downrate_int = 2  # Step the sentry at every Nth iteration
        self.stats = hello_utils.LoopStats(loop_name='RobotSafetyThread',target_loop_rate=self.update_rate_hz)
        self.shutdown_flag = threading.Event()
        self.running=False
        self.logger = logging.getLogger()
        if self.robot.params['use_monitor']:
            self.robot.monitor.startup()
        if self.robot.params['use_collision_manager']:
            self.robot.collision.startup()
        self.titr=0
    def run(self):
        while not self.shutdown_flag.is_set():
            self.stats.mark_loop_start()
            self.running = True
            ts = time.time()

            if self.robot.params['use_monitor']:
                if (self.titr % self.monitor_downrate_int) == 0:
                    self.robot.monitor.step()
            if self.robot.params['use_collision_manager']:
                    self.robot.collision.step()
            if self.robot.params['use_sentry']:
                if (self.titr % self.sentry_downrate_int) == 0:
                    self.robot.head.step_sentry(self.robot)
                    self.robot.base.step_sentry(self.robot)
                    self.robot.arm.step_sentry(self.robot)
                    self.robot.lift.step_sentry(self.robot)
                    self.robot.end_of_arm.step_sentry(self.robot)
            self.stats.mark_loop_end()
            if not self.shutdown_flag.is_set():
                time.sleep(self.stats.get_loop_sleep_time())
        self.logger.debug('Shutting down RobotSafetyThread')


class Robot(Device):
    """
    API to the Stretch RE1 Robot
    """
    def __init__(self):
        Device.__init__(self, 'robot')
        self.monitor = RobotMonitor(self)
        self.collision = RobotCollision(self)
        self.dirty_push_command = False
        self.lock = threading.RLock() #Prevent status thread from triggering motor sync prematurely
        self.status = {'pimu': {}, 'base': {}, 'lift': {}, 'arm': {}, 'head': {}, 'wacc': {}, 'end_of_arm': {}}

        self.pimu=pimu.Pimu()
        self.status['pimu']=self.pimu.status

        self.base=base.Base()
        self.status['base']=self.base.status

        self.lift=lift.Lift()
        self.status['lift']=self.lift.status

        self.arm=arm.Arm()
        self.status['arm']=self.arm.status

        self.head=head.Head()
        self.status['head']=self.head.status

        if 'custom_wacc' in self.params:
            module_name = self.params['custom_wacc']['py_module_name']
            class_name = self.params['custom_wacc']['py_class_name']
            self.wacc=getattr(importlib.import_module(module_name), class_name)(self)
        else:
            self.wacc=wacc.Wacc()
        self.status['wacc']=self.wacc.status


        tool_name = self.params['tool']
        module_name = self.robot_params[tool_name]['py_module_name']
        class_name = self.robot_params[tool_name]['py_class_name']
        self.end_of_arm = getattr(importlib.import_module(module_name), class_name)()
        self.status['end_of_arm'] = self.end_of_arm.status

        self.devices={ 'pimu':self.pimu, 'base':self.base, 'lift':self.lift, 'arm': self.arm, 'head': self.head, 'wacc':self.wacc, 'end_of_arm':self.end_of_arm}
        self.safety_thread=None
        self.dxl_thread=None
        self.non_dxl_thread=None

    # ###########  Device Methods #############

    def startup(self):
        """
        To be called once after class instantiation.
        Prepares devices for communications and motion
        """
        self.logger.debug('Starting up Robot {0} of batch {1}'.format(self.params['serial_no'], self.params['batch_name']))
        for k in self.devices.keys():
            if self.devices[k] is not None:
                if not self.devices[k].startup():
                    pass


        # Register the signal handlers
        signal.signal(signal.SIGTERM, hello_utils.thread_service_shutdown)
        signal.signal(signal.SIGINT, hello_utils.thread_service_shutdown)

        self.safety_thread = RobotSafetyThread(self)
        self.safety_thread.setDaemon(True)
        self.safety_thread.start()

        self.dxl_thread = DXLStatusThread(self)
        self.dxl_thread.setDaemon(True)
        self.dxl_thread.start()

        self.non_dxl_thread = NonDXLStatusThread(self)
        self.non_dxl_thread.setDaemon(True)
        self.non_dxl_thread.start()

        # Wait for status reading threads to start reading data
        ts=time.time()
        while not self.safety_thread.running and not self.dxl_thread.running and not self.non_dxl_thread.running and time.time()-ts<3.0:
           time.sleep(0.1)

    def stop(self):
        """
        To be called once before exiting a program
        Cleanly stops down motion and communication
        """
        print('---- Shutting down robot ----')
        if self.non_dxl_thread is not None:
            self.non_dxl_thread.shutdown_flag.set()
            self.non_dxl_thread.join(1)
        if self.dxl_thread is not None:
            self.dxl_thread.shutdown_flag.set()
            self.dxl_thread.join(1)
        if self.safety_thread is not None:
            self.safety_thread.shutdown_flag.set()
            self.safety_thread.join(1)
        for k in self.devices.keys():
            if self.devices[k] is not None:
                self.logger.debug('Shutting down %s'%k)
                self.devices[k].stop()
        self.logger.debug('---- Shutdown complete ----')

    def get_status(self):
        """
        Thread safe and atomic read of current Robot status data
        Returns as a dict.
        """
        with self.lock:
            return self.status.copy()

    def pretty_print(self):
        s=self.get_status()
        print('##################### HELLO ROBOT ##################### ')
        print('Time',time.time())
        print('Serial No',self.params['serial_no'])
        print('Batch', self.params['batch_name'])
        hello_utils.pretty_print_dict('Status',s)

    def push_command(self):
        """
        Cause all queued up RPC commands to be sent down to Devices
        """
        with self.lock:
            loop = asyncio.get_event_loop()
            loop.run_until_complete(asyncio.gather(
                self.wacc.push_command_async(),
                self.base.push_command_async(),
                self.lift.push_command_async(),
                self.arm.push_command_async(),
                self.pimu.push_command_async()))
            self.pimu.trigger_motor_sync()

# ##################Home and Stow #######################################

    def is_calibrated(self):
        """
        Returns true if homing-calibration has been run all joints that require it
        """
        ready = self.lift.motor.status['pos_calibrated']
        ready = ready and self.arm.motor.status['pos_calibrated']
        for j in self.end_of_arm.joints:
            req = self.end_of_arm.motors[j].params['req_calibration'] and not self.end_of_arm.motors[j].is_calibrated
            ready = ready and not req
        return ready

    def stow(self):
        """
        Cause the robot to move to its stow position
        Blocking.
        """
        self.head.move_to('head_pan', self.params['stow']['head_pan'])
        self.head.move_to('head_tilt',self.params['stow']['head_tilt'])

        lift_stowed=False
        pos_lift = self.params['stow']['lift']
        if 'stow' in self.end_of_arm.params:  # Allow tool defined stow position to take precedence
            if 'lift' in self.end_of_arm.params['stow']:
                pos_lift = self.end_of_arm.params['stow']['lift']
        if self.lift.status['pos']<=self.params['stow']['lift']: #Needs to come up before bring in arm
            print('--------- Stowing Lift ----')
            self.lift.move_to(pos_lift)
            self.push_command()
            time.sleep(0.25)
            ts = time.time()
            while not self.lift.motor.status['near_pos_setpoint'] and time.time() - ts < 3.0:
                time.sleep(0.1)
            lift_stowed=True

        #Bring in arm before bring down
        print('--------- Stowing Arm ----')
        pos_arm = self.params['stow']['arm']
        if 'stow' in self.end_of_arm.params:  # Allow tool defined stow position to take precedence
            if 'arm' in self.end_of_arm.params['stow']:
                pos_arm = self.end_of_arm.params['stow']['arm']

        self.arm.move_to(pos_arm)
        self.push_command()
        time.sleep(0.25)
        ts = time.time()
        while not self.arm.motor.status['near_pos_setpoint'] and time.time() - ts < 3.0:
            time.sleep(0.1)

        self.end_of_arm.stow()
        time.sleep(0.25)


        #Now bring lift down
        if not lift_stowed:
            print('--------- Stowing Lift ----')
            self.lift.move_to(pos_lift)
            self.push_command()
            time.sleep(0.25)
            ts = time.time()
            while not self.lift.motor.status['near_pos_setpoint'] and time.time() - ts < 10.0:
                time.sleep(0.1)

        #Make sure wrist yaw is done before exiting
        while self.end_of_arm.motors['wrist_yaw'].motor.is_moving():
            time.sleep(0.1)

    def home(self):
        """
        Cause the robot to home its joints by moving to hardstops
        Blocking.
        """
        if self.head is not None:
            print('--------- Homing Head ----')
            self.head.home()

        # Home the lift
        if self.lift is not None:
            print('--------- Homing Lift ----')
            self.lift.home()

        # Home the arm
        if self.arm is not None:
            print('--------- Homing Arm ----')
            self.arm.home()

        # Home the end-of-arm
        if self.end_of_arm is not None:
            self.end_of_arm.home()
        #Let user know it is done
        self.pimu.trigger_beep()
        self.push_command()


