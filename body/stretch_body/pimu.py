from __future__ import print_function
from stretch_body.transport import *
from stretch_body.device import Device
from stretch_body.hello_utils import *
import textwrap
import threading
import psutil
import logging
import time

RPC_SET_PIMU_CONFIG = 1
RPC_REPLY_PIMU_CONFIG = 2
RPC_GET_PIMU_STATUS = 3
RPC_REPLY_PIMU_STATUS = 4
RPC_SET_PIMU_TRIGGER = 5
RPC_REPLY_PIMU_TRIGGER = 6
RPC_GET_PIMU_BOARD_INFO =7
RPC_REPLY_PIMU_BOARD_INFO =8
RPC_SET_MOTOR_SYNC =9
RPC_REPLY_MOTOR_SYNC =10

STATE_AT_CLIFF_0= 1
STATE_AT_CLIFF_1= 2
STATE_AT_CLIFF_2= 4
STATE_AT_CLIFF_3= 8
STATE_RUNSTOP_EVENT= 16
STATE_CLIFF_EVENT= 32
STATE_FAN_ON =64
STATE_BUZZER_ON= 128
STATE_LOW_VOLTAGE_ALERT=256
STATE_OVER_TILT_ALERT= 512
STATE_HIGH_CURRENT_ALERT= 1024

TRIGGER_BOARD_RESET=  1
TRIGGER_RUNSTOP_RESET=  2
TRIGGER_CLIFF_EVENT_RESET= 4
TRIGGER_BUZZER_ON=  8
TRIGGER_BUZZER_OFF=  16
TRIGGER_FAN_ON=  32
TRIGGER_FAN_OFF = 64
TRIGGER_IMU_RESET =128
TRIGGER_RUNSTOP_ON= 256
TRIGGER_BEEP =512
# ######################## PIMU #################################

"""
The PIMU is the power and IMU Arduino board in the base
"""


class IMU(Device):
    """
    API to the Stretch RE1 IMU found in the base
    """
    def __init__(self):
        Device.__init__(self, 'imu')
        #pitch; //-180 to 180, rolls over
        #roll; //-90 to  90, rolls over at 180
        #heading; //0-360.0, rolls over
        self.status={'ax':0,'ay':0,'az':0,'gx':0,'gy':0,'gz':0,'mx':0,'my':0,'mz':0,'roll':0,'pitch':0,'heading':0,'timestamp':0,'qw':0,'qx':0,'qy':0,'qz':0,'bump':0}

    def get_status(self):
        s=self.status.copy()
        return s

    def get_quaternion(self):
        return [self.status['qw'],self.status['qx'],self.status['qy'],self.status['qz']]
    # ####################################################

    def pretty_print(self):
        print('----------IMU -------------')
        print('AX (m/s^2)', self.status['ax'])
        print('AY (m/s^2)', self.status['ay'])
        print('AZ (m/s^2)', self.status['az'])
        print('GX (rad/s)', self.status['gx'])
        print('GY (rad/s)', self.status['gy'])
        print('GZ (rad/s)', self.status['gz'])
        print('MX (uTesla)', self.status['mx'])
        print('MY (uTesla)', self.status['my'])
        print('MZ (uTesla)', self.status['mz'])
        print('QW', self.status['qw'])
        print('QX', self.status['qx'])
        print('QY', self.status['qy'])
        print('QZ', self.status['qz'])
        print('Roll (deg)', rad_to_deg(self.status['roll']))
        print('Pitch (deg)', rad_to_deg(self.status['pitch']))
        print('Heading (deg)', rad_to_deg(self.status['heading']))
        print('Bump', self.status['bump'])
        print('Timestamp', self.status['timestamp'])
        print('-----------------------')

    #Called by transport thread
    def unpack_status(self, rpc):
        # take in an array of bytes
        # this needs to exactly match the C struct format
        self.status['ax']=  rpc.unpack_float_t()
        self.status['ay'] = rpc.unpack_float_t()
        self.status['az'] = rpc.unpack_float_t()
        self.status['gx'] = rpc.unpack_float_t()
        self.status['gy'] = rpc.unpack_float_t()
        self.status['gz'] = rpc.unpack_float_t()
        self.status['mx'] = rpc.unpack_float_t()
        self.status['my'] = rpc.unpack_float_t()
        self.status['mz'] = rpc.unpack_float_t()
        self.status['roll'] = deg_to_rad(rpc.unpack_float_t())
        self.status['pitch'] = deg_to_rad(rpc.unpack_float_t())
        self.status['heading'] = deg_to_rad(rpc.unpack_float_t())
        self.status['qw'] = rpc.unpack_float_t()
        self.status['qx'] = rpc.unpack_float_t()
        self.status['qy'] = rpc.unpack_float_t()
        self.status['qz'] = rpc.unpack_float_t()
        self.status['bump'] = rpc.unpack_float_t()
        self.status['timestamp'] = self.timestamp.set(rpc.unpack_uint32_t())

class Pimu(Device):
    """
    API to the Stretch RE1 Power and IMU board (Pimu)
    """
    def __init__(self, event_reset=False):
        Device.__init__(self, 'pimu')
        self.imu = IMU()
        self.config = self.params['config']
        self._dirty_config = True
        self._dirty_trigger = False
        self.frame_id_last = None
        self.frame_id_base = 0
        self.name = 'hello-pimu'
        self.transport = Transport(usb_name='/dev/hello-pimu', logger=self.logger)
        self.status = {'voltage': 0, 'current': 0, 'temp': 0,'cpu_temp': 0, 'cliff_range':[0,0,0,0], 'frame_id': 0,
                       'timestamp': 0,'at_cliff':[False,False,False,False], 'runstop_event': False, 'bump_event_cnt': 0,
                       'cliff_event': False, 'fan_on': False, 'buzzer_on': False, 'low_voltage_alert':False,'high_current_alert':False,'over_tilt_alert':False,
                       'imu': self.imu.status,'debug':0,'state':0,
                       'transport': self.transport.status}
        self._trigger=0
        self.ts_last_fan_on=None
        self.fan_on_last=False
        #Reset PIMU state so that Ctrl-C and re-instantiate Pimu class is efficient way to get out of an event
        if event_reset:
            self.runstop_event_reset()
            self.cliff_event_reset()

        self.board_info = {'board_version': None, 'firmware_version': None, 'protocol_version': None}
        self.valid_firmware_protocol = 'p0'
        self.hw_valid = False

    # ###########  Device Methods #############

    def startup(self):
        self.hw_valid=self.transport.startup()
        if self.hw_valid:
            self.transport.execute_rpc(RPCRequest(id=RPC_GET_PIMU_BOARD_INFO, callback=self.rpc_board_info_reply))
            # Check that protocol matches
            if not(self.valid_firmware_protocol == self.board_info['protocol_version']):
                protocol_msg = """
                ----------------
                Firmware protocol mismatch on {0}.
                Protocol on board is {1}.
                Valid protocol is: {2}.
                Disabling device.
                Please upgrade the firmware and/or version of Stretch Body.
                ----------------
                """.format(self.name, self.board_info['protocol_version'], self.valid_firmware_protocol)
                self.logger.warn(textwrap.dedent(protocol_msg))
                self.hw_valid=False
                self.transport.stop()
        if self.hw_valid:
            self.push_command()
            self.pull_status()
        return self.hw_valid

    def stop(self):
        if self.hw_valid:
            self.hw_valid = False
            self.set_fan_off()
            self.push_command()
            self.transport.stop()

    # ###############################################

    def push_command(self):
        if self.hw_valid:
            rpcs=self._queue_command()
            for r in rpcs:
                self.transport.execute_rpc(r)

    async def push_command_async(self):
        if self.hw_valid:
            rpcs=self._queue_command()
            for r in rpcs:
                await self.transport.execute_rpc_async(r)

    def pull_status(self):
        if self.hw_valid:
            rpc = RPCRequest(id=RPC_GET_PIMU_STATUS, callback=self.rpc_status_reply)
            self.transport.execute_rpc(rpc)

    async def pull_status_async(self):
        if self.hw_valid:
            rpc=RPCRequest(id=RPC_GET_PIMU_STATUS, callback=self.rpc_status_reply)
            await self.transport.execute_rpc_async(rpc)

    def _queue_command(self):
        rpcs = []
        if self._dirty_config:
            rpcs.append(RPCRequest(id=RPC_SET_PIMU_CONFIG, callback=self.rpc_config_reply))
            self.pack_config(rpcs[-1])
            self._dirty_config = False

        if self._dirty_trigger:
            rpcs.append(RPCRequest(id=RPC_SET_PIMU_TRIGGER, callback=self.rpc_trigger_reply))
            self.pack_trigger(rpcs[-1])
            self._trigger = 0
            self._dirty_trigger = False
        return rpcs
    # ######################################################

    def pretty_print(self):
        print('------ Pimu -----')
        print('Voltage',self.status['voltage'])
        print('Current', self.status['current'])
        print('CPU Temp',self.status['cpu_temp'])
        print('Board Temp', self.status['temp'])
        print('State', self.status['state'])
        print('At Cliff', self.status['at_cliff'])
        print('Cliff Range', self.status['cliff_range'])
        print('Cliff Event', self.status['cliff_event'])
        print('Runstop Event', self.status['runstop_event'])
        print('Bump Event Cnt', self.status['bump_event_cnt'])
        print('Fan On', self.status['fan_on'])
        print('Buzzer On', self.status['buzzer_on'])
        print('Low Voltage Alert', self.status['low_voltage_alert'])
        print('High Current Alert', self.status['high_current_alert'])
        print('Over Tilt Alert',self.status['over_tilt_alert'])
        print('Debug', self.status['debug'])
        print('Timestamp', self.status['timestamp'])
        print('Board version:',self.board_info['board_version'])
        print('Firmware version:', self.board_info['firmware_version'])
        self.imu.pretty_print()

    # ####################### User Functions #######################################################

    def runstop_event_reset(self):
        """
        Reset the robot runstop, allowing motion to continue
        """
        self._trigger=self._trigger | TRIGGER_RUNSTOP_RESET
        self._dirty_trigger=True

    def runstop_event_trigger(self):
        """
        Trigger the robot runstop, stopping motion
        """
        self._trigger=self._trigger | TRIGGER_RUNSTOP_ON
        self._dirty_trigger=True

    def trigger_beep(self):
        """
        Generate a single short beep
        """
        self._trigger=self._trigger | TRIGGER_BEEP
        self._dirty_trigger=True

    # ####################### Utility functions ####################################################
    def imu_reset(self):
        self._trigger=self._trigger | TRIGGER_IMU_RESET
        self._dirty_trigger=True

    def trigger_motor_sync(self):
        #Execute immediately rather than queue
        if self.hw_valid:
            self.transport.execute_rpc(RPCRequest(id=RPC_SET_MOTOR_SYNC, callback=self.rpc_motor_sync_reply))

    def set_fan_on(self):
        self._trigger=self._trigger | TRIGGER_FAN_ON
        self._dirty_trigger=True

    def set_fan_off(self):
        self._trigger=self._trigger | TRIGGER_FAN_OFF
        self._dirty_trigger=True

    def set_buzzer_on(self):
        self._trigger=self._trigger | TRIGGER_BUZZER_ON
        self._dirty_trigger=True

    def set_buzzer_off(self):
        self._trigger=self._trigger | TRIGGER_BUZZER_OFF
        self._dirty_trigger=True

    def board_reset(self):
        self._trigger=self._trigger | TRIGGER_BOARD_RESET
        self._dirty_trigger=True

    def cliff_event_reset(self):
        self._trigger=self._trigger | TRIGGER_CLIFF_EVENT_RESET
        self._dirty_trigger=True

    # ########### Sensor Calibration #################
    def get_voltage(self,raw):
        raw_to_V = 20.0/1024 #10bit adc, 0-20V per 0-3.3V reading
        return raw*raw_to_V


    def get_temp(self,raw):
        raw_to_mV = 3300/1024.0
        mV = raw*raw_to_mV - 400 #400mV at 0C per spec
        C = mV/19.5 #19.5mV per C per spec
        return C

    def get_current(self,raw):
        raw_to_mV = 3300 / 1024.0
        mV = raw * raw_to_mV
        mA = mV/.408 # conversion per circuit
        return mA/1000.0
    # ################Data Packing #####################
    def unpack_trigger_reply(self,reply):
        rpc = RPCReply(reply)
        tt=rpc.unpack_uint32_t() #discard

    def unpack_board_info(self,reply):
        rpc = RPCReply(reply)
        self.board_info['board_version'] = rpc.unpack_string_t(20)
        self.board_info['firmware_version'] = rpc.unpack_string_t(20)
        self.board_info['protocol_version'] = self.board_info['firmware_version'][self.board_info['firmware_version'].rfind('p'):]

    def unpack_status(self,reply):
        rpc = RPCReply(reply)
        self.imu.unpack_status(rpc)
        self.status['voltage']=self.get_voltage(rpc.unpack_float_t())
        self.status['current'] = self.get_current(rpc.unpack_float_t())
        self.status['temp'] = self.get_temp(rpc.unpack_float_t())
        for i in range(4):
            self.status['cliff_range'][i]=rpc.unpack_float_t()
        self.status['state'] = rpc.unpack_uint32_t()
        self.status['at_cliff']=[]
        self.status['at_cliff'].append((self.status['state'] & STATE_AT_CLIFF_0) != 0)
        self.status['at_cliff'].append((self.status['state'] & STATE_AT_CLIFF_1) != 0)
        self.status['at_cliff'].append((self.status['state'] & STATE_AT_CLIFF_2) != 0)
        self.status['at_cliff'].append((self.status['state'] & STATE_AT_CLIFF_3) != 0)
        self.status['runstop_event'] = (self.status['state'] & STATE_RUNSTOP_EVENT) != 0
        self.status['cliff_event'] = (self.status['state'] & STATE_CLIFF_EVENT) != 0
        self.status['fan_on'] = (self.status['state'] & STATE_FAN_ON) != 0
        self.status['buzzer_on'] = (self.status['state'] & STATE_BUZZER_ON) != 0
        self.status['low_voltage_alert'] = (self.status['state'] & STATE_LOW_VOLTAGE_ALERT) != 0
        self.status['high_current_alert'] = (self.status['state'] & STATE_HIGH_CURRENT_ALERT) != 0
        self.status['over_tilt_alert'] = (self.status['state'] & STATE_OVER_TILT_ALERT) != 0
        self.status['timestamp'] = self.timestamp.set(rpc.unpack_uint32_t())
        self.status['bump_event_cnt'] = rpc.unpack_uint16_t()
        self.status['debug'] = rpc.unpack_float_t()
        self.status['cpu_temp']=self.get_cpu_temp()


    def pack_config(self,rpc):
        for i in range(4):
            rpc.pack_float_t(self.config['cliff_zero'][i])
        rpc.pack_float_t(self.config['cliff_thresh'])
        rpc.pack_float_t(self.config['cliff_LPF'])
        rpc.pack_float_t(self.config['voltage_LPF'])
        rpc.pack_float_t(self.config['current_LPF'])
        rpc.pack_float_t(self.config['temp_LPF'])
        rpc.pack_uint8_t(self.config['stop_at_cliff'])
        rpc.pack_uint8_t(self.config['stop_at_runstop'])
        rpc.pack_uint8_t(self.config['stop_at_tilt'])
        rpc.pack_uint8_t(self.config['stop_at_low_voltage'])
        rpc.pack_uint8_t(self.config['stop_at_high_current'])
        for i in range(3):
            rpc.pack_float_t(self.config['mag_offsets'][i])
        for i in range(9):
            rpc.pack_float_t(self.config['mag_softiron_matrix'][i])
        for i in range(3):
            rpc.pack_float_t(self.config['gyro_zero_offsets'][i])
        rpc.pack_float_t(self.config['rate_gyro_vector_scale'])
        rpc.pack_float_t(self.config['gravity_vector_scale'])
        rpc.pack_float_t(self.config['accel_LPF'])
        rpc.pack_float_t(self.config['bump_thresh'])
        rpc.pack_float_t(self.config['low_voltage_alert'])
        rpc.pack_float_t(self.config['high_current_alert'])
        rpc.pack_float_t(self.config['over_tilt_alert'])


    def pack_trigger(self,rpc):
        rpc.pack_uint32_t(self._trigger)


    # ################Transport Callbacks #####################

    def rpc_motor_sync_reply(self,reply):
        if reply[0] != RPC_REPLY_MOTOR_SYNC:
            print('Error RPC_REPLY_MOTOR_SYNC', reply[0])

    def rpc_config_reply(self,reply):
        if reply[0] != RPC_REPLY_PIMU_CONFIG:
            print('Error RPC_REPLY_PIMU_CONFIG', reply[0])

    def rpc_board_info_reply(self,reply):
        if reply[0] == RPC_REPLY_PIMU_BOARD_INFO:
            self.unpack_board_info(reply[1:])
        else:
            print('Error RPC_REPLY_PIMU_BOARD_INFO', reply[0])

    def rpc_trigger_reply(self,reply):
        if reply[0] != RPC_REPLY_PIMU_TRIGGER:
            print('Error RPC_REPLY_PIMU_TRIGGER', reply[0])
        else:
            self.unpack_trigger_reply(reply[1:])

    def rpc_status_reply(self,reply):
        if reply[0] == RPC_REPLY_PIMU_STATUS:
            self.unpack_status(reply[1:])
        else:
            print('Error RPC_REPLY_PIMU_STATUS', reply[0])

    # ################ Sentry #####################
    def get_cpu_temp(self):
        cpu_temp = 0
        try:
            t = psutil.sensors_temperatures()['coretemp']
            for c in t:
                cpu_temp = max(cpu_temp, c.current)
        except KeyError: #May not be available on virtual machines
            cpu_temp=25.0
        return cpu_temp

    def step_sentry(self,robot):
        if self.hw_valid and self.robot_params['robot_sentry']['base_fan_control']:
            #Manage CPU temp using the mobile base fan
            #See https://www.intel.com/content/www/us/en/support/articles/000005946/intel-nuc.html
            cpu_temp=self.get_cpu_temp()
            if cpu_temp>self.params['base_fan_on']:
                if self.ts_last_fan_on is None or time.time()-self.ts_last_fan_on>3.0: #Will turn itself off if don't refresh command
                    self.set_fan_on()
                    self.push_command()
                    self.ts_last_fan_on = time.time()
                if  not self.status['fan_on']:
                    self.logger.debug('Base fan turned on')

            if self.fan_on_last and not self.status['fan_on']:
                self.logger.debug('Base fan turned off')

            if cpu_temp<self.params['base_fan_off']and self.status['fan_on']:
                self.set_fan_off()
                self.push_command()
            self.fan_on_last = self.status['fan_on']
