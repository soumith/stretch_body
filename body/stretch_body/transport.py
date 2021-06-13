from __future__ import print_function
import serial
import time
import struct
import array as arr
import fcntl
import logging
import aioserial
import asyncio
import traceback
import threading

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

TODO
* make queue and immediate rpc thread safe / atomic (sync and async)
* Tests for thread safety

Handle an asynchronous RPC transaction.

An RPC transaction consists of
* Send [RPC_ID, N bytes of data] --> uC
* Receive reply=[RPC_ACK_ID, M bytes of data]<--uC
* Call rpc_callback(reply)

For performance reasons the max size of data send down/up from the uC is 64 bytes
Therefore, we break RPC data into frames of size 32 bytes max.
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
    def __init__(self,port_name,ser,logger):
        self.ser=ser
        self.logger=logger
        self.port_name=port_name
        self.framer = CobbsFraming()

    async def step_transaction(self,rpc): #Handle a single RPC transaction
        """
        :param rpc: buffer of rpc data to send
        :param rpc_callback: function to call upon completion of transaction
        :return:
        """
        try:
            #Transmit the RPC request
            await self._step_frame(id=RPC_START_NEW_RPC,ack=RPC_ACK_NEW_RPC,data=[])
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
            reply = []
            ts=time.time()
            while True:
                buf_rx, nr =await self._step_frame(id=RPC_GET_BLOCK, ack=RPC_ACK_GET_BLOCK_LAST, data=[], ack_alt=RPC_ACK_GET_BLOCK_MORE)
                reply = reply + buf_rx[1:nr]
                if time.time()-ts>1.0:#Timeout in case very corrupted comms
                    raise TransportError
                if buf_rx[0]==RPC_ACK_GET_BLOCK_LAST:
                    rpc.callback(arr.array('B', reply))  # Now process the reply with the callback
                    break
        except TransportError as e:
            print("TransportError: %s : %s" % (self.port_name, str(e)))
        except serial.SerialTimeoutException as e:
            print("SerialTimeoutException: %s : %s" % (self.port_name, str(e)))
        except serial.SerialException as e:
            print("SerialException: %s : %s" % (self.port_name, str(e)))
        except TypeError as e:
            print(traceback.format_exc())
            print("TypeError: %s : %s" % (self.port_name, str(e)))

    async def _recv_framed_data(self):
        """
        Receive a single frame back
        Frame is Cobbs encoder with packet marker of \x00
        :return: CRC valid, num bytes received, buffer of decoded data
        """
        rbuf= await self.ser.read_until_async(expected= b'\x00', size=RPC_BLOCK_SIZE*2)
        if len(rbuf)==0 or rbuf[-1]!=0:
            self.logger.warn('Warning: Transport invalid read %d bytes during _recv_framed_data' % len(rbuf))
            return 0, 0, [0]
        crc_ok, nr, decoded_data = self.framer.decode_data(rbuf[:-1])
        return crc_ok, nr, decoded_data

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

    def __init__(self, port_name, ser, logger):
        self.ser = ser
        self.logger = logger
        self.port_name = port_name
        self.framer = CobbsFraming()

    def step_transaction(self, rpc):  # Handle a single RPC transaction
        """
        :param rpc: buffer of rpc data to send
        :param rpc_callback: function to call upon completion of transaction
        :return:
        """
        try:
            # Transmit the RPC request
            self._step_frame(id=RPC_START_NEW_RPC, ack=RPC_ACK_NEW_RPC, data=[])
            ntx = 0
            while ntx < rpc.nbytes:
                nb = min(rpc.nbytes - ntx, RPC_BLOCK_SIZE)  # Num bytes to send
                b = rpc.data[ntx:ntx + nb]
                ntx = ntx + nb
                if ntx == rpc.nbytes:  # Last block
                    self._step_frame(id=RPC_SEND_BLOCK_LAST, ack=RPC_ACK_SEND_BLOCK_LAST, data=b)
                else:
                    self._step_frame(id=RPC_SEND_BLOCK_MORE, ack=RPC_ACK_SEND_BLOCK_MORE, data=b)
            # Now receive RPC reply in one or more frames
            reply = []
            ts = time.time()
            while True:
                buf_rx, nr = self._step_frame(id=RPC_GET_BLOCK, ack=RPC_ACK_GET_BLOCK_LAST, data=[], ack_alt=RPC_ACK_GET_BLOCK_MORE)
                reply = reply + buf_rx[1:nr]
                if time.time() - ts > 1.0:  # Timeout in case very corrupted comms
                    raise TransportError
                if buf_rx[0] == RPC_ACK_GET_BLOCK_LAST:
                    rpc.callback(arr.array('B', reply))  # Now process the reply with the callback
                    break
        except TransportError as e:
            print("TransportError: %s : %s" % (self.port_name, str(e)))
        except serial.SerialTimeoutException as e:
            print("SerialTimeoutException: %s : %s" % (self.port_name, str(e)))
        except serial.SerialException as e:
            print("SerialException: %s : %s" % (self.port_name, str(e)))
        except TypeError as e:
            print(traceback.format_exc())
            print("TypeError: %s : %s" % (self.port_name, str(e)))

    def _recv_framed_data(self):
        """
        Receive a single frame back
        Frame is Cobbs encoder with packet marker of \x00
        :return: CRC valid, num bytes received, buffer of decoded data
        """
        rbuf= self.ser.read_until(expected= b'\x00', size=RPC_BLOCK_SIZE*2)
        if len(rbuf)==0 or rbuf[-1]!=0:
            self.logger.warn('Warning: Transport invalid read %d bytes during _recv_framed_data' % len(rbuf))
            return 0, 0, [0]
        crc_ok, nr, decoded_data = self.framer.decode_data(rbuf[:-1])
        return crc_ok, nr, decoded_data


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

    Aioserial supports the standard  pySerial interface as well as async versions of the pySerial interface. 
    This enables Transport to support both synchronous and asynchromous calls. 
    Devices can use standard pySerial for non-timing critical transactions.
    They can use the asyncio interfaces for timing critical transactions where blocking on the RPC call is not desirable

    """
    def __init__(self,port_name, logger=logging.getLogger()):
        self.port_name=port_name
        self.logger=logger
        self.logger.debug('Starting TransportConnection on: ' + self.port_name)
        self.status = {}
        self.hw_valid=False
        self.lock=threading.Lock()

    def startup(self):
        try:
            self.ser = aioserial.AioSerial(port=self.port_name, write_timeout=1.0, timeout=1.0)
            self.hw_valid = True
            if self.ser.isOpen():
                try:
                    fcntl.flock(self.ser.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                except IOError:
                    self.logger.error('Port %s is busy. Check if another Stretch Body process is already running' % self.port_name)
                    self.ser.close()
                    self.hw_valid=False
        except serial.SerialException as e:
            self.logger.error("SerialException({0}): {1}".format(e.errno, e.strerror))
            self.hw_valid=False
        if not self.hw_valid:
            self.logger.warn('Unable to open serial port for device %s' % self.port_name)
        else:
            self.async_handler=AsyncTransactionHandler(port_name=self.port_name, ser=self.ser, logger=self.logger)
            self.sync_handler=SyncTransactionHandler(port_name=self.port_name, ser=self.ser, logger=self.logger)
        return self.hw_valid

    def stop(self):
        if self.hw_valid:
            self.logger.debug('Shutting down TransportConnection on: ' + self.port_name)
            self.ser.close()
            self.hw_valid = False
    # #################################
    def execute_rpc(self, rpc):
        if self.hw_valid:
            with self.lock:
                self.sync_handler.step_transaction(rpc)

    async def execute_rpc_async(self, rpc):
        if self.hw_valid:
            # Acquire the lock in a worker thread, suspending us while waiting.
            # See https://stackoverflow.com/questions/63420413/how-to-use-threading-lock-in-async-function-while-object-can-be-accessed-from-mu
            await asyncio.get_event_loop().run_in_executor(None, self.lock.acquire)
            await self.async_handler.step_transaction(rpc)
            self.lock.release()
