from __future__ import print_function
from stretch_body.transport import *
from stretch_body.device import Device
from stretch_body.hello_utils import *
import textwrap
import threading
import sys

RPC_SET_COMMAND = 1
RPC_REPLY_COMMAND = 2
RPC_GET_STATUS = 3
RPC_REPLY_STATUS = 4
RPC_SET_GAINS = 5
RPC_REPLY_GAINS = 6
RPC_LOAD_TEST =7
RPC_REPLY_LOAD_TEST =8
RPC_SET_TRIGGER = 9
RPC_REPLY_SET_TRIGGER =10
RPC_SET_ENC_CALIB =11
RPC_REPLY_ENC_CALIB =12
RPC_READ_GAINS_FROM_FLASH =13
RPC_REPLY_READ_GAINS_FROM_FLASH =14
RPC_SET_MENU_ON =15
RPC_REPLY_MENU_ON=16
RPC_GET_STEPPER_BOARD_INFO =17
RPC_REPLY_STEPPER_BOARD_INFO =18
RPC_SET_MOTION_LIMITS=19
RPC_REPLY_MOTION_LIMITS =20

MODE_SAFETY=0
MODE_FREEWHEEL=1
MODE_HOLD=2
MODE_POS_PID=3
MODE_VEL_PID=4
MODE_POS_TRAJ=5
MODE_VEL_TRAJ=6
MODE_CURRENT=7
MODE_POS_TRAJ_INCR=8

DIAG_POS_CALIBRATED =1         #Has a pos zero RPC been recieved since powerup
DIAG_RUNSTOP_ON =2             #Is controller in runstop mode
DIAG_NEAR_POS_SETPOINT =4      #Is pos controller within gains.pAs_d of setpoint
DIAG_NEAR_VEL_SETPOINT =8     #Is vel controller within gains.vAs_d of setpoint
DIAG_IS_MOVING =16             #Is measured velocity greater than gains.vAs_d
DIAG_AT_CURRENT_LIMIT =32      #Is controller current saturated
DIAG_IS_MG_ACCELERATING =64   #Is controler motion generator acceleration non-zero
DIAG_IS_MG_MOVING =128         #Is controller motion generator velocity non-zero
DIAG_CALIBRATION_RCVD = 256     #Is calibration table in flash
DIAG_IN_GUARDED_EVENT = 512     # Guarded event occured during motion
DIAG_IN_SAFETY_EVENT = 1024      #Is it forced into safety mode
DIAG_WAITING_ON_SYNC = 2048     #Command received but no sync yet

CONFIG_SAFETY_HOLD =1           #Hold position in safety mode? Otherwise freewheel
CONFIG_ENABLE_RUNSTOP =2        #Recognize runstop signal?
CONFIG_ENABLE_SYNC_MODE =4      #Commands are synchronized from digital trigger
CONFIG_ENABLE_GUARDED_MODE=8    #Stops on current threshold
CONFIG_FLIP_ENCODER_POLARITY=16
CONFIG_FLIP_EFFORT_POLARITY=32

TRIGGER_MARK_POS = 1
TRIGGER_RESET_MOTION_GEN = 2
TRIGGER_BOARD_RESET = 4
TRIGGER_WRITE_GAINS_TO_FLASH = 8
TRIGGER_RESET_POS_CALIBRATED = 16
TRIGGER_POS_CALIBRATED = 32


class Stepper(Device):
    """
    API to the Stretch RE1 stepper board
    """
    def __init__(self, usb):
        name = usb[5:]
        Device.__init__(self,name)
        self.usb=usb
        self.transport = Transport(port_name=self.usb, logger=self.logger)

        self._command = {'mode':0, 'x_des':0,'v_des':0,'a_des':0,'stiffness':1.0,'i_feedforward':0.0,'i_contact_pos':0,'i_contact_neg':0,'incr_trigger':0}
        self.status = {'mode': 0, 'effort': 0, 'current':0,'pos': 0, 'vel': 0, 'err':0,'diag': 0,'timestamp': 0, 'debug':0,'guarded_event':0,
                       'transport': self.transport.status,'pos_calibrated':0,'runstop_on':0,'near_pos_setpoint':0,'near_vel_setpoint':0,
                       'is_moving':0,'at_current_limit':0,'is_mg_accelerating':0,'is_mg_moving':0, 'calibration_rcvd': 0, 'in_guarded_event':0,
                       'in_safety_event':0,'waiting_on_sync':0}
        self.board_info={'board_version':None, 'firmware_version':None,'protocol_version':None}
        self.mode_names={MODE_SAFETY:'MODE_SAFETY', MODE_FREEWHEEL:'MODE_FREEWHEEL',MODE_HOLD:'MODE_HOLD',MODE_POS_PID:'MODE_POS_PID',
                         MODE_VEL_PID:'MODE_VEL_PID',MODE_POS_TRAJ:'MODE_POS_TRAJ',MODE_VEL_TRAJ:'MODE_VEL_TRAJ',MODE_CURRENT:'MODE_CURRENT', MODE_POS_TRAJ_INCR:'MODE_POS_TRAJ_INCR'}
        self.motion_limits=[0,0]

        self._dirty_command = False
        self._dirty_gains = False
        self._dirty_trigger = False
        self._dirty_read_gains_from_flash=False
        self._dirty_motion_limits=False
        self._dirty_load_test=False
        self._trigger=0
        self._trigger_data=0
        self.load_test_payload = arr.array('B', range(256)) * 4
        self.valid_firmware_protocol='p0'
        self.hw_valid=False
        self.gains = self.params['gains'].copy()

    # ###########  Device Methods #############
    def startup(self):
        self.hw_valid=self.transport.startup()
        if self.hw_valid:
            self.transport.execute_rpc(RPCRequest(id=RPC_GET_STEPPER_BOARD_INFO,callback=self.rpc_board_info_reply))
            #Check that protocol matches
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
                self.logger.warning(textwrap.dedent(protocol_msg))
                self.hw_valid=False
                self.transport.stop()
        if self.hw_valid:
            self.enable_safety()
            self._dirty_gains = True
            self.pull_status()
            self.push_command()
        return self.hw_valid


    #Configure control mode prior to calling this on process shutdown (or default to freewheel)
    def stop(self):
        if not self.hw_valid:
            return
        self.hw_valid = False
        self.logger.debug('Shutting down Stepper on: ' + self.usb)
        self.enable_safety()
        self.push_command()
        self.transport.stop()

    # ###############################################

    def push_command(self):
        if self.hw_valid:
            self.transport.execute_rpc(self._get_queued_commands())

    async def push_command_async(self):
        if self.hw_valid:
            await self.transport.execute_rpc_async(self._get_queued_commands())

    def pull_status(self):
        if self.hw_valid:
            rpc = RPCRequest(id=RPC_GET_STATUS, callback=self.rpc_status_reply)
            self.transport.execute_rpc(rpc)

    async def pull_status_async(self):
        if self.hw_valid:
            rpc=RPCRequest(id=RPC_GET_STATUS, callback=self.rpc_status_reply)
            await self.transport.execute_rpc_async(rpc)

    # ###############################################

    def _get_queued_commands(self):
        rpcs=[]
        if self._dirty_load_test:
            rpcs.append(RPCRequest(id=RPC_LOAD_TEST, callback=self.rpc_load_test_reply))
            rpcs[-1].pack_n_bytes(self.load_test_payload,1024)
            self._dirty_load_test=False

        if self._dirty_read_gains_from_flash:
            rpcs.append(RPCRequest(id=RPC_READ_GAINS_FROM_FLASH, callback=self.rpc_read_gains_from_flash_reply))
            self._dirty_read_gains_from_flash = False

        if self._dirty_motion_limits:
            rpcs.append(RPCRequest(id=RPC_SET_MOTION_LIMITS, callback=self.rpc_motion_limits_reply))
            self.pack_motion_limits(rpcs[-1])
            self._dirty_motion_limits = False

        if self._dirty_trigger:
            rpcs.append(RPCRequest(id=RPC_SET_TRIGGER, callback=self.rpc_trigger_reply))
            self.pack_trigger(rpcs[-1])
            self._trigger=0
            self._dirty_trigger = False

        if self._dirty_gains:
            rpcs.append(RPCRequest(id=RPC_SET_GAINS, callback=self.rpc_gains_reply))
            self.pack_gains(rpcs[-1])
            self._dirty_gains=False

        if self._dirty_command:
            rpcs.append(RPCRequest(id=RPC_SET_COMMAND, callback=self.rpc_command_reply))
            self.pack_command(rpcs[-1])
            self._dirty_command=False
        return rpcs

    def pretty_print(self):
        print('-----------')
        print('Mode',self.mode_names[self.status['mode']])
        print('x_des (rad)', self._command['x_des'], '(deg)',rad_to_deg(self._command['x_des']))
        print('v_des (rad)', self._command['v_des'], '(deg)',rad_to_deg(self._command['v_des']))
        print('a_des (rad)', self._command['a_des'], '(deg)',rad_to_deg(self._command['a_des']))
        print('Stiffness',self._command['stiffness'])
        print('Feedforward', self._command['i_feedforward'])
        print('Pos (rad)', self.status['pos'], '(deg)',rad_to_deg(self.status['pos']))
        print('Vel (rad/s)', self.status['vel'], '(deg)',rad_to_deg(self.status['vel']))
        print('Effort', self.status['effort'])
        print('Current (A)', self.status['current'])
        print('Error (deg)', rad_to_deg(self.status['err']))
        print('Debug', self.status['debug'])
        print('Guarded Events:', self.status['guarded_event'])
        print('Diag', self.status['diag'])
        print('       Position Calibrated:', self.status['pos_calibrated'])
        print('       Runstop on:', self.status['runstop_on'])
        print('       Near Pos Setpoint:', self.status['near_pos_setpoint'])
        print('       Near Vel Setpoint:', self.status['near_vel_setpoint'])
        print('       Is Moving:', self.status['is_moving'])
        print('       At Current Limit:', self.status['at_current_limit'])
        print('       Is MG Accelerating:', self.status['is_mg_accelerating'])
        print('       Is MG Moving:', self.status['is_mg_moving'])
        print('       Encoder Calibration in Flash:', self.status['calibration_rcvd'])
        print('       In Guarded Event:', self.status['in_guarded_event'])
        print('       In Safety Event:', self.status['in_safety_event'])
        print('       Waiting on Sync:', self.status['waiting_on_sync'])
        print('Timestamp', self.status['timestamp'])
        #print('Read error', self.transport.status['read_error'])
        print('Board version:', self.board_info['board_version'])
        print('Firmware version:', self.board_info['firmware_version'])
    # ###########################################################################

    def set_load_test(self):
        self._dirty_load_test=True

    def set_motion_limits(self,limit_neg, limit_pos):
        if limit_neg!=self.motion_limits[0] or limit_pos!=self.motion_limits[1]:
            self.motion_limits=[limit_neg, limit_pos]
            self._dirty_motion_limits=True

    def set_gains(self,g):
        self.gains=g.copy()
        self._dirty_gains = True

    def write_gains_to_YAML(self):
        self.params['gains']=self.gains.copy()
        self.write_device_params(self.name,self.params)

    def write_gains_to_flash(self):
        self._trigger = self._trigger | TRIGGER_WRITE_GAINS_TO_FLASH
        self._dirty_trigger = True

    def read_gains_from_flash(self):
        self._dirty_read_gains_from_flash=True

    def board_reset(self):
        self._trigger = self._trigger | TRIGGER_BOARD_RESET
        self._dirty_trigger=True

    def mark_position(self,x):
        if self.status['mode']!=MODE_SAFETY:
            print('Can not mark position. Must be in MODE_SAFETY for',self.usb)
            return

        self._trigger_data=x
        self._trigger = self._trigger | TRIGGER_MARK_POS
        self._dirty_trigger=True

    def reset_motion_gen(self):
        self._trigger = self._trigger | TRIGGER_RESET_MOTION_GEN
        self._dirty_trigger = True

    def reset_pos_calibrated(self):
        self._trigger = self._trigger | TRIGGER_RESET_POS_CALIBRATED
        self._dirty_trigger = True

    def set_pos_calibrated(self):
        self._trigger = self._trigger | TRIGGER_POS_CALIBRATED
        self._dirty_trigger = True

    # ###########################################################################
    def enable_safety(self):
        self.set_command(mode=MODE_SAFETY)

    def enable_freewheel(self):
        self.set_command(mode=MODE_FREEWHEEL)

    def enable_hold(self):
        self.set_command(mode=MODE_HOLD)

    def enable_vel_pid(self):
        self.set_command(mode=MODE_VEL_PID, v_des=0)

    def enable_pos_pid(self):
        self.set_command(mode=MODE_POS_PID, x_des=self.status['pos'])

    def enable_vel_traj(self):
        self.set_command(mode=MODE_VEL_TRAJ, v_des=0)

    def enable_pos_traj(self):
        self.set_command(mode=MODE_POS_TRAJ, x_des=self.status['pos'])

    def enable_pos_traj_incr(self):
        self.set_command(mode=MODE_POS_TRAJ_INCR, x_des=0)

    def enable_current(self):
        self.set_command(mode=MODE_CURRENT, i_des=0)

    def enable_sync_mode(self):
        self.gains['enable_sync_mode'] = 1
        self._dirty_gains = 1

    def disable_sync_mode(self):
        self.gains['enable_sync_mode']=0
        self._dirty_gains=1

    def enable_runstop(self):
        self.gains['enable_runstop'] = 1
        self._dirty_gains = 1

    def disable_runstop(self):
        self.gains['enable_runstop']=0
        self._dirty_gains=1

    def enable_guarded_mode(self):
        self.gains['enable_guarded_mode'] = 1
        self._dirty_gains = 1

    def disable_guarded_mode(self):
        self.gains['enable_guarded_mode'] = 0
        self._dirty_gains = 1

    #Primary interface to controlling the stepper
    #YAML defaults are used if values not provided
    #This allows user to override defaults every control cycle and then easily revert to defaults
    def set_command(self,mode=None, x_des=None, v_des=None, a_des=None,i_des=None, stiffness=None,i_feedforward=None, i_contact_pos=None, i_contact_neg=None  ):
        if mode is not None:
            self._command['mode'] = mode

        if x_des is not None:
            self._command['x_des'] = x_des
            if self._command['mode'] == MODE_POS_TRAJ_INCR:
                self._command['incr_trigger'] = (self._command['incr_trigger']+1)%255

        if v_des is not None:
            self._command['v_des'] = v_des
        else:
            if mode == MODE_VEL_PID or mode == MODE_VEL_TRAJ:
                self._command['v_des'] = 0
            else:
                self._command['v_des'] = self.params['motion']['vel']

        if a_des is not None:
            self._command['a_des'] = a_des
        else:
            self._command['a_des'] = self.params['motion']['accel']

        if stiffness is not None:
            self._command['stiffness'] = max(0, min(1.0, stiffness))
        else:
            self._command['stiffness'] =1.0

        if i_feedforward is not None:
            self._command['i_feedforward'] = i_feedforward
        else:
            self._command['i_feedforward'] = 0

        if i_des is not None and mode == MODE_CURRENT:
            self._command['i_feedforward'] =i_des

        if i_contact_pos is not None:
            self._command['i_contact_pos'] = i_contact_pos
        else:
            self._command['i_contact_pos']=self.params['gains']['i_contact_pos']

        if i_contact_neg is not None:
            self._command['i_contact_neg'] = i_contact_neg
        else:
            self._command['i_contact_neg'] = self.params['gains']['i_contact_neg']

        self._dirty_command=True


    def wait_until_at_setpoint(self,timeout=15.0):
        ts = time.time()
        self.pull_status()
        while not self.status['near_pos_setpoint'] and time.time() - ts < timeout:
            time.sleep(0.1)
            self.pull_status()

    def current_to_effort(self,i_A):
        mA_per_tick = (3300 / 255) / (10 * 0.1)
        effort = (i_A * 1000.0) / mA_per_tick
        return min(255,max(-255,int(effort)))

    def effort_to_current(self,e):
        mA_per_tick = (3300 / 255) / (10 * 0.1)
        return e * mA_per_tick / 1000.0

    #Very rough, not accounting for motor dynamics / temp / etc
    def current_to_torque(self,i):
        k_t = self.params['holding_torque']/self.params['rated_current'] #N-m/A
        return i*k_t

    def torque_to_current(self, tq):
        k_t = self.params['holding_torque'] / self.params['rated_current']  # N-m/A
        return tq/k_t

        # ####################### Encoder Calibration ######################

    def get_chip_id(self):
        self.turn_menu_interface_on()
        time.sleep(0.5)
        cid = self.menu_transaction('b', do_print=False)[0][:-2]
        self.turn_rpc_interface_on()
        time.sleep(0.5)
        return cid

    def read_encoder_calibration_from_YAML(self):
        device_name=self.usb[5:]
        sn=self.robot_params[device_name]['serial_no']
        fn='calibration_steppers/'+device_name+'_'+sn+'.yaml'
        enc_data=read_fleet_yaml(fn)
        return enc_data

    def write_encoder_calibration_to_YAML(self,data):
        device_name = self.usb[5:]
        sn = self.robot_params[device_name]['serial_no']
        fn = 'calibration_steppers/'+device_name + '_' + sn + '.yaml'
        write_fleet_yaml(fn,data)

    def read_encoder_calibration_from_flash(self):
        self.turn_menu_interface_on()
        time.sleep(0.5)
        print('Reading encoder calibration...')
        e = self.menu_transaction('q',do_print=False)[19]
        self.turn_rpc_interface_on()
        self.push_command()
        print('Reseting board')
        self.board_reset()
        self.push_command()
        e = e[:-4]  # We now have string of floats, convert to list of floats
        enc_calib = []
        while len(e):
            ff = e.find(',')
            if ff != -1:
                enc_calib.append(float(e[:ff]))
                e = e[ff + 2:]
            else:
                enc_calib.append(float(e))
                e = []
        if len(enc_calib)==16384:
            print('Successful read of encoder calibration')
        else:
            print('Failed to read encoder calibration')
        return enc_calib

    def write_encoder_calibration_to_flash(self,data):
        if not self.hw_valid:
            return
        #This will take a few seconds. Blocks until complete.
        if len(data)!=16384:
            print('Bad encoder data')
        else:
            print('Writing encoder calibration...')
            for p in range(256):
                if p%10==0:
                    sys.stdout.write('.')
                    sys.stdout.flush()
                rpc=RPCRequest(RPC_SET_ENC_CALIB,self.rpc_enc_calib_reply)
                rpc.pack_uint8_t(p)
                for i in range(64):
                    rpc.pack_float_t(data[p*64+i])
                self.transport.execute_rpc(rpc)
            print('')

    def rpc_enc_calib_reply(self,reply):
        if reply[0] != RPC_REPLY_ENC_CALIB:
            print('Error RPC_REPLY_ENC_CALIB', reply[0])

    # ######################Menu Inteface ################################3


    def turn_rpc_interface_on(self):
        self.menu_transaction('zyx')


    def turn_menu_interface_on(self):
        if self.hw_valid:
            self.transport.execute_rpc(RPCRequest(RPC_SET_MENU_ON,self.rpc_menu_on_reply))

    def print_menu(self):
        self.menu_transaction('m')

    def menu_transaction(self,x,do_print=True):
        do_print=True
        reply = []
        if self.hw_valid:
            self.transport.ser.write(str.encode(x))
            time.sleep(0.1)
            while self.transport.ser.inWaiting():
                r=self.transport.ser.readline()
                if do_print:
                    print(r, end=' ')
                reply.append(r)
        print('Menu transaction',x)
        return reply

    # ################ RPC Callbacks #####################

    def rpc_load_test_reply(self, reply):
        if reply[0] == RPC_REPLY_LOAD_TEST:
            pass_test=True
            d = reply[1:]
            for i in range(1024):
                if d[i] != self.load_test_payload[(i + 1) % 1024]:
                    print('Load test bad data', d[i], self.load_test_payload[(i + 1) % 1024])
                    pass_test=False
            self.load_test_payload = d
            if pass_test:
                print('Load Test data valid')
                return True
            else:
                return False
        else:
            print('Error RPC_REPLY_LOAD_TEST', reply[0])
            return False

    def rpc_board_info_reply(self, reply):
        if reply[0] == RPC_REPLY_STEPPER_BOARD_INFO:
            self.unpack_board_info(reply[1:])
            return True
        else:
            print('Error RPC_REPLY_STEPPER_BOARD_INFO', reply[0])
            return False

    def rpc_gains_reply(self, reply):
        if reply[0] != RPC_REPLY_GAINS:
            print('Error RPC_REPLY_GAINS', reply[0])
            return False
        return True

    def rpc_trigger_reply(self, reply):
        if reply[0] != RPC_REPLY_SET_TRIGGER:
            print('Error RPC_REPLY_SET_TRIGGER', reply[0])
            return False
        return True

    def rpc_command_reply(self, reply):
        if reply[0] != RPC_REPLY_COMMAND:
            print('Error RPC_REPLY_COMMAND', reply[0])
            return False
        return True

    def rpc_motion_limits_reply(self, reply):
        if reply[0] != RPC_REPLY_MOTION_LIMITS:
            print('Error RPC_REPLY_MOTION_LIMITS', reply[0])
            return False
        return True

    def rpc_menu_on_reply(self, reply):
        if reply[0] != RPC_REPLY_MENU_ON:
            print('Error RPC_REPLY_MENU_ON', reply[0])
            return False
        return True

    def rpc_status_reply(self, reply):
        if reply[0] == RPC_REPLY_STATUS:
            self.unpack_status(reply[1:])
            return True
        else:
            print('Error RPC_REPLY_STATUS', reply[0])
            return False

    def rpc_read_gains_from_flash_reply(self, reply):
        if reply[0] == RPC_REPLY_READ_GAINS_FROM_FLASH:
            self.unpack_gains(reply[1:])
            return True
        else:
            print('Error RPC_REPLY_READ_GAINS_FROM_FLASH', reply[0])
            return False

    def unpack_board_info(self,reply):
        rpc=RPCReply(reply)
        self.board_info['board_version'] = rpc.unpack_string_t(20).strip('\x00')
        self.board_info['firmware_version'] = rpc.unpack_string_t(20).strip('\x00')
        self.board_info['protocol_version'] = self.board_info['firmware_version'][self.board_info['firmware_version'].rfind('p'):]

    def unpack_status(self,reply):
        rpc = RPCReply(reply)
        self.status['mode']=rpc.unpack_uint8_t()
        self.status['effort'] = rpc.unpack_float_t()
        self.status['current']=self.effort_to_current(self.status['effort'])
        self.status['pos'] = rpc.unpack_double_t()
        self.status['vel'] = rpc.unpack_float_t()
        self.status['err'] = rpc.unpack_float_t()
        self.status['diag'] = rpc.unpack_uint32_t()
        self.status['timestamp'] = self.timestamp.set(rpc.unpack_uint32_t())
        self.status['debug'] = rpc.unpack_float_t()
        self.status['guarded_event'] = rpc.unpack_uint32_t()
        self.status['pos_calibrated'] =self.status['diag'] & DIAG_POS_CALIBRATED > 0
        self.status['runstop_on'] =self.status['diag'] & DIAG_RUNSTOP_ON > 0
        self.status['near_pos_setpoint'] =self.status['diag'] & DIAG_NEAR_POS_SETPOINT > 0
        self.status['near_vel_setpoint'] = self.status['diag'] & DIAG_NEAR_VEL_SETPOINT > 0
        self.status['is_moving'] =self.status['diag'] & DIAG_IS_MOVING > 0
        self.status['at_current_limit'] =self.status['diag'] & DIAG_AT_CURRENT_LIMIT > 0
        self.status['is_mg_accelerating'] = self.status['diag'] & DIAG_IS_MG_ACCELERATING > 0
        self.status['is_mg_moving'] =self.status['diag'] & DIAG_IS_MG_MOVING > 0
        self.status['calibration_rcvd'] = self.status['diag'] & DIAG_CALIBRATION_RCVD > 0
        self.status['in_guarded_event'] = self.status['diag'] & DIAG_IN_GUARDED_EVENT > 0
        self.status['in_safety_event'] = self.status['diag'] & DIAG_IN_SAFETY_EVENT > 0
        self.status['waiting_on_sync'] = self.status['diag'] & DIAG_WAITING_ON_SYNC > 0



    def unpack_gains(self,reply):
        rpc = RPCReply(reply)
        self.gains['pKp_d'] = rpc.unpack_float_t()
        self.gains['pKi_d'] = rpc.unpack_float_t()
        self.gains['pKd_d'] = rpc.unpack_float_t()
        self.gains['pLPF'] = rpc.unpack_float_t()
        self.gains['pKi_limit'] = rpc.unpack_float_t()
        self.gains['vKp_d'] = rpc.unpack_float_t()
        self.gains['vKi_d'] = rpc.unpack_float_t()
        self.gains['vKd_d'] = rpc.unpack_float_t()
        self.gains['vLPF'] = rpc.unpack_float_t()
        self.gains['vKi_limit'] = rpc.unpack_float_t()
        self.gains['vTe_d'] = rpc.unpack_float_t()
        self.gains['iMax_pos'] = rpc.unpack_float_t()
        self.gains['iMax_neg'] = rpc.unpack_float_t()
        self.gains['phase_advance_d'] = rpc.unpack_float_t()
        self.gains['pos_near_setpoint_d'] = rpc.unpack_float_t()
        self.gains['vel_near_setpoint_d'] = rpc.unpack_float_t()
        self.gains['vel_status_LPF'] = rpc.unpack_float_t()
        self.gains['effort_LPF'] = rpc.unpack_float_t()
        self.gains['safety_stiffness'] = rpc.unpack_float_t()
        self.gains['i_safety_feedforward'] = rpc.unpack_float_t()
        config = rpc.unpack_uint8_t()
        self.gains['safety_hold']= int(config & CONFIG_SAFETY_HOLD>0)
        self.gains['enable_runstop'] = int(config & CONFIG_ENABLE_RUNSTOP>0)
        self.gains['enable_sync_mode'] = int(config & CONFIG_ENABLE_SYNC_MODE>0)
        self.gains['enable_guarded_mode'] = int(config & CONFIG_ENABLE_GUARDED_MODE > 0)
        self.gains['flip_encoder_polarity'] = int(config & CONFIG_FLIP_ENCODER_POLARITY > 0)
        self.gains['flip_effort_polarity'] = int(config & CONFIG_FLIP_EFFORT_POLARITY > 0)

    def pack_motion_limits(self,rpc):
        rpc.pack_float_t(self.motion_limits[0])
        rpc.pack_float_t(self.motion_limits[1])

    def pack_command(self,rpc):
        rpc.pack_uint8_t(self._command['mode'])
        rpc.pack_float_t(self._command['x_des'])
        rpc.pack_float_t(self._command['v_des'])
        rpc.pack_float_t(self._command['a_des'])
        rpc.pack_float_t(self._command['stiffness'])
        rpc.pack_float_t(self._command['i_feedforward'])
        rpc.pack_float_t(self._command['i_contact_pos'])
        rpc.pack_float_t(self._command['i_contact_neg'])
        rpc.pack_uint8_t(self._command['incr_trigger'])

    def pack_gains(self,rpc):
        rpc.pack_float_t(self.gains['pKp_d'])
        rpc.pack_float_t(self.gains['pKi_d'])
        rpc.pack_float_t(self.gains['pKd_d'])
        rpc.pack_float_t(self.gains['pLPF'])
        rpc.pack_float_t(self.gains['pKi_limit'])
        rpc.pack_float_t(self.gains['vKp_d'])
        rpc.pack_float_t(self.gains['vKi_d'])
        rpc.pack_float_t(self.gains['vKd_d'])
        rpc.pack_float_t(self.gains['vLPF'])
        rpc.pack_float_t(self.gains['vKi_limit'])
        rpc.pack_float_t(self.gains['vTe_d'])
        rpc.pack_float_t(self.gains['iMax_pos'])
        rpc.pack_float_t(self.gains['iMax_neg'])
        rpc.pack_float_t(self.gains['phase_advance_d'])
        rpc.pack_float_t(self.gains['pos_near_setpoint_d'])
        rpc.pack_float_t(self.gains['vel_near_setpoint_d'])
        rpc.pack_float_t(self.gains['vel_status_LPF'])
        rpc.pack_float_t(self.gains['effort_LPF'])
        rpc.pack_float_t(self.gains['safety_stiffness'])
        rpc.pack_float_t(self.gains['i_safety_feedforward'])
        config=0
        if self.gains['safety_hold']:
            config=config | CONFIG_SAFETY_HOLD
        if self.gains['enable_runstop']:
            config=config | CONFIG_ENABLE_RUNSTOP
        if self.gains['enable_sync_mode']:
            config=config | CONFIG_ENABLE_SYNC_MODE
        if self.gains['enable_guarded_mode']:
            config=config | CONFIG_ENABLE_GUARDED_MODE
        if self.gains['flip_encoder_polarity']:
            config = config | CONFIG_FLIP_ENCODER_POLARITY
        if self.gains['flip_effort_polarity']:
            config = config | CONFIG_FLIP_EFFORT_POLARITY
        rpc.pack_uint8_t(config)


    def pack_trigger(self,rpc):
        rpc.pack_uint32_t(self._trigger)
        rpc.pack_float_t(self._trigger_data)

