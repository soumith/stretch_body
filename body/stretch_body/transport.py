from __future__ import print_function
import serial
import time
import struct
import array as arr
import fcntl
import logging
import aioserial
import traceback

# ######################################################
class TransportError(Exception):
    """Base class for exceptions in this module."""
    pass


RPC_START_NEW_RPC =100
RPC_ACK_NEW_RPC= 101

RPC_SEND_BLOCK_MORE = 102
RPC_ACK_SEND_BLOCK_MORE = 103
RPC_SEND_BLOCK_LAST = 104
RPC_ACK_SEND_BLOCK_LAST = 105

RPC_GET_BLOCK = 106
RPC_ACK_GET_BLOCK_MORE = 107
RPC_ACK_GET_BLOCK_LAST = 108

RPC_BLOCK_SIZE = 32
RPC_DATA_SIZE = 1024

dbg_on = 0
"""
Handle an asynchronous RPC transaction.

An RPC transaction consists of
* Send [RPC_ID, N bytes of data] --> uC
* Receive reply=[RPC_ACK_ID, M bytes of data]<--uC
* Call rpc_callback(reply)

For performance reasons the max size of data send down/up from the uC is 64 bytes
Therefore, we break RPC data into frames of size 64 bytes max.
We don't know apriori the amount of data being sent.
A frame transaction looks like:

Transmit data down:
* Send and initialization frame [RPC_START_NEW_RPC], Rcv back [RPC_ACK_NEW_RPC]
* Send zero or more data frames: Send [RPC_SEND_BLOCK_MORE, n bytes] ,Rcv [RPC_ACK_SEND_BLOCK_MORE]
* Send a terminating data frame: Send [RPC_SEND_BLOCK_LAST, n bytes], Rcv [RPC_ACK_SEND_BLOCK_LAST]

Receive data back:
* Send [RPC_GET_BLOCK], Rcv [RPC_ACK_GET_BLOCK_MORE, nybtes] or [RPC_ACK_GET_BLOCK_LAST, nbytes]
* Continue to recieve data back until get a RPC_ACK_GET_BLOCK_LAST

Each frame is also Cobbs encoded prior to transmission and Cobbs decoded upon receipt.

"""
# #############################################################################################
class AsyncTransactionHandler():
    """
    Handle an asynchronous RPC transaction.
    """
    def __init__(self,usb_name,ser,logger):
        self.ser=ser
        self.logger=logger
        self.usb_name=usb_name
        self.framer = CobbsFraming()

    async def step_transaction(self,rpc): #Handle a single RPC transaction
        """
        :param rpc: buffer of rpc data to send
        :param rpc_callback: function to call upon completion of transaction
        :return:
        """
        try:
            #Start new RPC transmission
            #rpc=list(rpc) #todo: cleanup types
            await self._step_frame(id=RPC_START_NEW_RPC,ack=RPC_ACK_NEW_RPC,data=[])
            #Send all RPC data down in one or more frames
            ntx = 0
            while ntx < rpc.nbytes:
                nb = min(rpc.nbytes - ntx, RPC_BLOCK_SIZE)  # Num bytes to send
                b = rpc.data[ntx:ntx + nb]
                ntx = ntx + nb
                if ntx == rpc.nbytes:  # Last block
                    await self._step_frame(id=RPC_SEND_BLOCK_LAST, ack=RPC_ACK_SEND_BLOCK_LAST,data=b)
                else:
                    await self._step_frame(id=RPC_SEND_BLOCK_MORE, ack=RPC_ACK_SEND_BLOCK_MORE,data=b)
            #Now receive RPC reply in one or more frames
            reply = []#arr.array('B')
            while True: #TODO: Timeout in case very corrupted comms
                buf_rx, nr =await self._step_frame(id=RPC_GET_BLOCK, ack=RPC_ACK_GET_BLOCK_LAST, data=[], ack_alt=RPC_ACK_GET_BLOCK_MORE)
                reply = reply + buf_rx[1:nr]
                if buf_rx[0]==RPC_ACK_GET_BLOCK_LAST:
                    break
            rpc.callback(arr.array('B', reply))  # Now process the reply with the callback
        except TransportError as e:
            print("TransportError: %s : %s" % (self.usb_name, str(e)))
        except serial.SerialTimeoutException as e:
            print("SerialTimeoutException: %s : %s" % (self.usb_name, str(e)))
        except serial.SerialException as e:
            print("SerialException: %s : %s" % (self.usb_name, str(e)))
        except TypeError as e:
            print(traceback.format_exc())
            print("TypeError: %s : %s" % (self.usb_name, str(e)))

    async def _recv_framed_data(self):
        """
        Receive a single frame back
        :return: CRC valid, num bytes received, buffer of decoded data
        """
        timeout = .2  # Was .05 but on heavy loads can get starved
        packet_marker = 0
        t_start = time.time()
        rx_buffer = []
        while ((time.time() - t_start) < timeout):
            #Todo: drop polling and just block so asynico event loop gets control
            nn = self.ser.inWaiting()
            if (nn > 0):
                rbuf = await self.ser.read_async(nn)
                nu = 0
                for byte_in in rbuf:
                    nu = nu + 1
                    if byte_in == packet_marker:
                        crc_ok, nr, decoded_data = self.framer.decode_data(rx_buffer)
                        if nu < nn:
                            self.logger.warn('Warning: Transport dropped %d bytes during _recv_framed_data' % (nn - nu))
                        return crc_ok, nr, decoded_data
                    else:
                        rx_buffer.append(byte_in)
        return 0, 0, []

    async def _step_frame(self,id, ack, data, ack_alt=None):
        """
        id: RPC Request ID
        ack: RPC acknowledge ID
        data: frame data to send to uC
        """
        buf_tx = arr.array('B', [id]+list(data))  # TODO, move to bytes
        encoded_data=arr.array('B',self.framer.encode_data(buf_tx ))
        await self.ser.write_async(encoded_data)
        crc, nr, buf_rx = await self._recv_framed_data()
        if crc==1 and (buf_rx[0]==ack or buf_rx[0]==ack_alt):
            return buf_rx, nr
        else:
            self.logger.warn('Transport Frame Error| CRC valid %d | Num bytes rcvd %d | Ack desired %d | Ack recvd %d'%(crc, nr, ack, buf_rx[0]))
            raise TransportError

# #############################################################################################
class SyncTransactionHandler():
    """
    Handle a synchronous RPC transaction.
    """

    def __init__(self, usb_name, ser, logger):
        self.ser = ser
        self.logger = logger
        self.usb_name = usb_name
        self.framer = CobbsFraming()

    def step_transaction(self, rpc):  # Handle a single RPC transaction
        """
        :param rpc: buffer of rpc data to send
        :param rpc_callback: function to call upon completion of transaction
        :return:
        """
        try:
            # Start new RPC transmission
            self._step_frame(id=RPC_START_NEW_RPC, ack=RPC_ACK_NEW_RPC, data=[])
            # Send all RPC data down in one or more frames
            ntx = 0
            while ntx < rpc.nbytes:
                nb = min(rpc.nbytes - ntx, RPC_BLOCK_SIZE)  # Num bytes to send
                b = rpc.data[ntx:ntx + nb]
                ntx += nb
                if ntx == rpc.nbytes:  # Last block
                    self._step_frame(id=RPC_SEND_BLOCK_LAST, ack=RPC_ACK_SEND_BLOCK_LAST, data=b)
                else:
                    self._step_frame(id=RPC_SEND_BLOCK_MORE, ack=RPC_ACK_SEND_BLOCK_MORE, data=b)
            # Now receive RPC reply in one or more frames
            reply = []
            while True:  # TODO: Timeout in case very corrupted comms
                buf_rx, nr = self._step_frame(id=RPC_GET_BLOCK, ack=RPC_ACK_GET_BLOCK_LAST, data=[],ack_alt=RPC_ACK_GET_BLOCK_MORE)
                reply = reply + buf_rx[1:nr]
                if buf_rx[0] == RPC_ACK_GET_BLOCK_LAST:
                    break
            rpc.callback(arr.array('B', reply))  # Now process the reply with the callback
        except TransportError as e:
            print("TransportError: %s : %s" % (self.usb_name, str(e)))
        except serial.SerialTimeoutException as e:
            print("SerialTimeoutException: %s : %s" % (self.usb_name, str(e)))
        except serial.SerialException as e:
            print("SerialException: %s : %s" % (self.usb_name, str(e)))
        except TypeError as e:
            print(traceback.format_exc())
            print("TypeError: %s : %s" % (self.usb_name, str(e)))

    def _recv_framed_data(self):
        """
        Receive a single frame back
        :return: CRC valid, num bytes received, buffer of decoded data
        """
        timeout = .2  # Was .05 but on heavy loads can get starved
        packet_marker = 0
        t_start = time.time()
        rx_buffer = []
        while ((time.time() - t_start) < timeout):
            # Todo: drop polling and just block so asynico event loop gets control
            nn = self.ser.inWaiting()
            if (nn > 0):
                rbuf = self.ser.read(nn)
                nu = 0
                for byte_in in rbuf:
                    nu = nu + 1
                    if byte_in == packet_marker:
                        crc_ok, nr, decoded_data = self.framer.decode_data(rx_buffer)
                        if nu < nn:
                            self.logger.warn('Warning: Transport dropped %d bytes during _recv_framed_data' % (nn - nu))
                        return crc_ok, nr, decoded_data
                    else:
                        rx_buffer.append(byte_in)
        return 0, 0, []

    def _step_frame(self, id, ack, data, ack_alt=None):
        """
        id: RPC Request ID
        ack: RPC acknowledge ID
        data: frame data to send to uC
        """
        buf_tx = arr.array('B', [id] + list(data))
        encoded_data = arr.array('B', self.framer.encode_data(buf_tx))
        self.ser.write(encoded_data)
        crc, nr, buf_rx = self._recv_framed_data()
        if crc == 1 and (buf_rx[0] == ack or buf_rx[0] == ack_alt):
            return buf_rx, nr
        else:
            self.logger.warn('Transport Frame Error| CRC valid %d | Num bytes rcvd %d | Ack desired %d | Ack recvd %d' % (crc, nr, ack, buf_rx[0]))
            raise TransportError

# #############################################################################################
#Based on COBS.h from
#https://github.com/bakercp/PacketSerial
#MIT License
#Copyright (c) 2017 Christopher Baker https://christopherbaker.net

class CobbsFraming():
    """
    Encoding for communications
    """
    def __init__(self):
        pass

    def encode_data(self, data):
        """
        Encode data (len nb bytes)
        Append CRC first
        Return buffer of encoded data
        """
        crc=self._calc_crc(data,len(data))
        data.append((crc>>8)&0xFF)
        data.append(crc & 0xFF)
        encoded_data=self._encode(data)
        encoded_data.append(0x00)
        return encoded_data

    def decode_data(self,data):
        """
        Decode data into decode buffer
        Check CRC
        Return crc ok, num bytes in decoded buffer
        """
        crc1, nr, decode_buffer = self._decode(data)
        crc2 = self._calc_crc(decode_buffer, nr)
        return crc1 == crc2, nr, decode_buffer

    # ######################################

    def _calc_crc(self, buf, nr): #Modbus CRC
        crc = 0xFFFF
        for i in range(nr):
            crc ^= buf[i]
            for i in range(8):
                if ((crc & 1) != 0):
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc


    def _encode(self,data):
        """
        Cobbs encode the data buffer of nb bytes
        Return encoded data
        """
        nb=len(data)
        read_index = 0
        write_index = 1
        code_index = 0
        code = 1
        encode_buffer=[0]*2*nb
        while (read_index < nb):
            if (data[read_index] == 0):
                encode_buffer[code_index] = code
                code = 1
                code_index = write_index
                write_index=write_index+1
                read_index=read_index+1
            else:
                encode_buffer[write_index]=data[read_index]
                read_index=read_index+1
                write_index=write_index+1
                code=code+1
                if (code == 0xFF):
                    encode_buffer[code_index] = code
                    code = 1
                    code_index = write_index
                    write_index=write_index+1
        encode_buffer[code_index] = code
        return encode_buffer[:write_index]

    def _decode(self, data):
        #return crc valid, num bytes in decode buffer, decoded data
        nb=len(data)
        if nb==0:
            return 0,0
        read_index=0
        write_index=0
        code =0
        decode_buffer=[0]*2*nb
        while read_index < nb:
            code = data[read_index]
            if (read_index + code > nb and code != 1):
                return 0,0
            read_index=read_index+1
            for i in range(1,code):
                decode_buffer[write_index]=data[read_index]
                read_index=read_index+1
                write_index=write_index+1
            if (code != 0xFF and read_index != nb):
                decode_buffer[write_index]=0
                write_index=write_index+1
        crc = (decode_buffer[write_index- 2]<<8)|decode_buffer[write_index-1]
        return crc, write_index-2, decode_buffer[:write_index]

# #############################################################################################
class RPCReply():
    def __init__(self,reply):
        self.data=reply

    def unpack_string_t(self, n):
        x= (struct.unpack(str(n) + 's', self.data[:n])[0].strip(b'\x00')).decode('utf-8')
        self.data=self.data[n:]
        return x

    def unpack_int32_t(self):
        x= struct.unpack('i', self.data[:4])[0]
        self.data=self.data[4:]
        return x

    def unpack_uint32_t(self):
        x= struct.unpack('I', self.data[:4])[0]
        self.data = self.data[4:]
        return x

    def unpack_int16_t(self):
        x= struct.unpack('h', self.data[:2])[0]
        self.data = self.data[2:]
        return x

    def unpack_uint16_t(self):
        x=  struct.unpack('H',self.data[:2])[0]
        self.data = self.data[2:]
        return x

    def unpack_uint8_t(self):
        x=  struct.unpack('B',self.data[:1])[0]
        self.data = self.data[1:]
        return x

    def unpack_float_t(self):
        x=  struct.unpack('f',self.data[:4])[0]
        self.data = self.data[4:]
        return x

    def unpack_double_t(self):
        x=  struct.unpack('d',self.data[:8])[0]
        self.data = self.data[8:]
        return x

class RPCRequest():
    def __init__(self,id, callback):
        self.id=id
        self.callback=callback
        self.data = arr.array('B', [0] * (RPC_DATA_SIZE + 1))
        self.data[0]=id
        self.nbytes=1 #Num bytes packed so far

    def pack_n_bytes(self,x,n):
        self.data[self.nbytes:self.nbytes+n]=x
        self.nbytes=self.nbytes+n

    def pack_string_t(self, x):
        n = len(x)
        struct.pack_into(str(n) + 's', self.data, self.nbytes, x)
        self.nbytes += n

    def pack_float_t(self,x):
        struct.pack_into('f', self.data, self.nbytes, x)
        self.nbytes += 4

    def pack_double_t(self,x):
        struct.pack_into('d',  self.data, self.nbytes, x)
        self.nbytes += 8

    def pack_int32_t(self,x):
        struct.pack_into('i',  self.data, self.nbytes, x)
        self.nbytes += 4

    def pack_uint32_t(self,x):
        struct.pack_into('I',  self.data, self.nbytes, x)
        self.nbytes += 4

    def pack_int16_t(self,x):
        struct.pack_into('h',  self.data, self.nbytes, x)
        self.nbytes += 2

    def pack_uint16_t(self,x):
        struct.pack_into('H',  self.data, self.nbytes, x)
        self.nbytes += 2

    def pack_uint8_t(self,x):
        struct.pack_into('B',  self.data, self.nbytes, x)
        self.nbytes += 1

# #############################################################################################

class Transport():
    """
    Handle serial communications to a Hello Robot USB device using pySerial and asyncio
    This class manages the serial devices, the RPC transaction handlers, as well as queueing of RPC requests

    An RPC can be executed immediately or queued to be batch executed later
    The queues are of type  push (sending command data down to uC) or pull (reading data from uC).
    The use of queues allows for synchronization of RPC commands

    The Transport opens a standard pySerial port as well as an asyncio serial port. This enables
    the use of standard pySerial for non-timing critical transactions while allowing for use of asyncio
    for timing critical transactions where blocking on the RPC call is not desirable

    """
    def __init__(self,usb_name, logger=logging.getLogger()):
        self.usb_name=usb_name
        self.logger=logger
        self.logger.debug('Starting TransportConnection on: ' + self.usb_name)
        self.rpc_queue_pull = []
        self.rpc_queue_push = []
        self.status = {}
        self.hw_valid=False

    def startup(self):
        try:
            self.ser_async = aioserial.AioSerial(port=self.usb_name)
            self.ser_sync = serial.Serial(self.usb_name, write_timeout=1.0)
            self.hw_valid = True
            if self.ser_sync.isOpen() and self.ser_async.isOpen():
                try:
                    fcntl.flock(self.ser_sync.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                except IOError:
                    self.logger.error('Port %s is busy. Check if another Stretch Body process is already running' % self.usb_name)
                    self.ser_sync.close()
                    self.ser_async.close()
                    self.hw_valid=False
        except serial.SerialException as e:
            self.logger.error("SerialException({0}): {1}".format(e.errno, e.strerror))
            self.hw_valid=False
        if not self.hw_valid:
            self.logger.warn('Unable to open serial port for device %s' % self.usb_name)
        else:
            self.async_handler=AsyncTransactionHandler(usb_name=self.usb_name, ser=self.ser_async, logger=self.logger)
            self.sync_handler=SyncTransactionHandler(usb_name=self.usb_name, ser=self.ser_sync, logger=self.logger)
        return self.hw_valid

    def stop(self):
        if self.hw_valid:
            self.logger.debug('Shutting down TransportConnection on: ' + self.usb_name)
            self.ser_sync.close()
            self.ser_async.close()
            self.hw_valid = False
    # #################################
    def execute_rpc(self, rpc):
        if self.hw_valid:
            self.sync_handler.step_transaction(rpc)

    async def execute_rpc_async(self, rpc):
        if self.hw_valid:
            await self.async_handler.step_transaction(rpc)

    def queue_rpc(self,rpc, pull):
        if self.hw_valid:
            if pull:
                self.rpc_queue_pull.append(rpc)
            else:
                self.rpc_queue_push.append(rpc)

    async def execute_queue_async(self,pull):
        if self.hw_valid:
            if pull:
                while len(self.rpc_queue_pull):
                    await self.execute_rpc_async(self.rpc_queue_pull.pop())
            else:
                while len(self.rpc_queue_push):
                    await self.execute_rpc_async(self.rpc_queue_push.pop())

    def execute_queue(self,pull):
        if self.hw_valid:
            if pull:
                while len(self.rpc_queue_pull):
                    self.execute_rpc(self.rpc_queue_pull.pop())
            else:
                while len(self.rpc_queue_push):
                    self.execute_rpc(self.rpc_queue_push.pop())

