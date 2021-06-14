from __future__ import print_function
from stretch_body.transport import *
from stretch_body.device import Device
import threading
import textwrap

RPC_SET_WACC_CONFIG = 1
RPC_REPLY_WACC_CONFIG = 2
RPC_GET_WACC_STATUS = 3
RPC_REPLY_WACC_STATUS = 4
RPC_SET_WACC_COMMAND = 5
RPC_REPLY_WACC_COMMAND = 6
RPC_GET_WACC_BOARD_INFO =7
RPC_REPLY_WACC_BOARD_INFO =8

TRIGGER_BOARD_RESET = 1

# ######################## WACC #################################

class Wacc(Device):
    """
    API to the Stretch RE1 wrist+accelerometer (Wacc) board
    The Wacc has:
    -- 3-axis accelerometer reported as Ax,Ay,and Az
    -- Two digital inputs D0, D1
    -- Two digital outputs D2, D3
    -- One analog input: A0
    -- A single tap count based on the accelerometer

    ext_status_cb: Callback to handle custom status data
    ext_command_cb: Callback to handle custom command data
    """

    def __init__(self, ext_status_cb=None, ext_command_cb=None):
        Device.__init__(self, 'wacc')
        self.ext_status_cb=ext_status_cb
        self.ext_command_cb=ext_command_cb
        self.config = self.params['config']
        self._dirty_config = True #Force push down
        self._dirty_command = False
        self._command = {'d2':0,'d3':0, 'trigger':0}
        self.name ='hello-wacc'
        self.transport = Transport(port_name='/dev/hello-wacc', logger=self.logger)
        self.status = { 'ax':0,'ay':0,'az':0,'a0':0,'d0':0,'d1':0, 'd2':0,'d3':0,'single_tap_count': 0, 'state':0, 'debug':0,
                       'timestamp': 0,
                       'transport': self.transport.status}
        self.ts_last=None
        self.board_info = {'board_version': None, 'firmware_version': None, 'protocol_version': None}
        self.valid_firmware_protocol = 'p0'
        self.hw_valid = False

    # ###########  Device Methods #############

    def startup(self):
        self.hw_valid=self.transport.startup()
        if self.hw_valid:
            self.transport.execute_rpc(RPCRequest(id=RPC_GET_WACC_BOARD_INFO, callback=self.rpc_board_info_reply))
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
            self.push_command()
            self.transport.stop()

    def set_D2(self,on):#0 or 1
        """
        Set the Digital Out 2 on the Wacc expansion header
        """
        self._command['d2']=bool(on)
        self._dirty_command = True

    def set_D3(self,on): #0 or 1
        """
        Set the Digital Out 3 on the Wacc expansion header
        """
        self._command['d3']=bool(on)
        self._dirty_command = True

    # ###############################################

    def push_command(self):
        if self.hw_valid:
            self.transport.execute_rpc(self._get_queued_commands())

    async def push_command_async(self):
        if self.hw_valid:
            await self.transport.execute_rpc(self._get_queued_commands())

    def pull_status(self):
        if self.hw_valid:
            rpc = RPCRequest(id=RPC_GET_WACC_STATUS, callback=self.rpc_status_reply)
            self.transport.execute_rpc(rpc)

    async def pull_status_async(self):
        if self.hw_valid:
            rpc = RPCRequest(id=RPC_GET_WACC_STATUS, callback=self.rpc_status_reply)
            await self.transport.execute_rpc_async(rpc)

    def _get_queued_commands(self):
        rpcs=[]
        if self._dirty_config:
            rpcs.append(RPCRequest(id=RPC_SET_WACC_CONFIG, callback=self.rpc_config_reply))
            self.pack_config(rpcs[-1])
            self._dirty_config = False

        if self._dirty_command:
            rpcs.append(RPCRequest(id=RPC_SET_WACC_COMMAND, callback=self.rpc_command_reply))
            self.pack_command(rpcs[-1])
            self._command['trigger'] =0
            self._dirty_command = False
        return rpcs

    # ############################################################################################

    def pretty_print(self):
        print('------------------------------')
        print('Ax (m/s^2)',self.status['ax'])
        print('Ay (m/s^2)', self.status['ay'])
        print('Az (m/s^2)', self.status['az'])
        print('A0', self.status['a0'])
        print('D0 (In)', self.status['d0'])
        print('D1 (In)', self.status['d1'])
        print('D2 (Out)', self.status['d2'])
        print('D3 (Out)', self.status['d3'])
        print('Single Tap Count', self.status['single_tap_count'])
        print('State ', self.status['state'])
        print('Debug',self.status['debug'])
        print('Timestamp', self.status['timestamp'])
        print('Board version:', self.board_info['board_version'])
        print('Firmware version:', self.board_info['firmware_version'])

    # ####################### Utility functions ####################################################
    def board_reset(self):
        self._command['trigger']=self._command['trigger']| TRIGGER_BOARD_RESET
        self._dirty_command=True

    # ################Data Packing #####################

    def unpack_board_info(self,reply):
        rpc = RPCReply(reply)
        self.board_info['board_version'] = rpc.unpack_string_t(20)
        self.board_info['firmware_version'] = rpc.unpack_string_t(20)
        self.board_info['protocol_version'] = self.board_info['firmware_version'][self.board_info['firmware_version'].rfind('p'):]


    def unpack_status(self,reply):
        rpc = RPCReply(reply)
        if self.ext_status_cb is not None:
            self.ext_status_cb(rpc) #TODO: Document change in interface for ext_status_cb from v0.1.x
        self.status['ax'] = rpc.unpack_float_t()
        self.status['ay'] = rpc.unpack_float_t()
        self.status['az'] = rpc.unpack_float_t()
        self.status['a0'] = rpc.unpack_int16_t()
        self.status['d0'] = rpc.unpack_uint8_t()
        self.status['d1'] = rpc.unpack_uint8_t()
        self.status['d2'] = rpc.unpack_uint8_t()
        self.status['d3'] = rpc.unpack_uint8_t()
        self.status['single_tap_count'] = rpc.unpack_uint32_t()
        self.status['state'] = rpc.unpack_uint32_t()
        self.status['timestamp'] = self.timestamp.set(rpc.unpack_uint32_t())
        self.status['debug'] = rpc.unpack_uint32_t()

    def pack_command(self,rpc):
        if self.ext_command_cb is not None:  # Pack custom data first
            self.ext_command_cb(rpc)  #TODO: Document change in interface for ext_status_cb from v0.1.x
        rpc.pack_uint8_t(self._command['d2'])
        rpc.pack_uint8_t(self._command['d3'])
        rpc.pack_uint32_t(self._command['trigger'])

    def pack_config(self,rpc):
        rpc.pack_uint8_t(self.config['accel_range_g'])
        rpc.pack_float_t(self.config['accel_LPF'])
        rpc.pack_float_t(self.config['ana_LPF'])
        rpc.pack_uint8_t(self.config['accel_single_tap_dur'])
        rpc.pack_uint8_t(self.config['accel_single_tap_thresh'])
        rpc.pack_float_t(self.config['accel_gravity_scale'])


    # ################Transport Callbacks #####################
    def rpc_board_info_reply(self,reply):
        if reply[0] == RPC_REPLY_WACC_BOARD_INFO:
            self.unpack_board_info(reply[1:])
        else:
            print('Error RPC_REPLY_WACC_BOARD_INFO', reply[0])

    def rpc_command_reply(self,reply):
        if reply[0] != RPC_REPLY_WACC_COMMAND:
            print('Error RPC_REPLY_WACC_COMMAND', reply[0])

    def rpc_config_reply(self,reply):
        if reply[0] != RPC_REPLY_WACC_CONFIG:
            print('Error RPC_REPLY_WACC_CONFIG', reply[0])

    def rpc_status_reply(self,reply):
        if reply[0] == RPC_REPLY_WACC_STATUS:
            self.unpack_status(reply[1:])
        else:
            print('Error RPC_REPLY_WACC_STATUS', reply[0])







