# -*- coding: utf-8 -*-
####!/bin/env python
"""
Four-channel USB Voltage and Thermocouple DAQ driver,
Resolution: 14-bit
Sampleling rate: 8000 Hz max
Range: +/- 50 V to +/- 10 mV, in 3 steps per decade (1,2.5,5)
build in cold junction compensation (CJC) for thermocouples

Reference:
DI-245 Communication Protocol
www.dataq.com/resources/pdfs/support_articles/DI-245-protocol.pdf

COM Port Communication Settings:
Baud rate: 115200, Data bits: 8, Stop bits: 1, Parity: none

Installing the DI-245 driver package and connecting DI-245 hardware to the
host computer’s USB port results in a COM port being hooked by the operating
system and assigned to the DI-245 device.

Multiple DI-245 devices may be connected to the same PC without additional
driver installations, which results in a unique COM port number assignment to
each by the operating system.

The DI-245 employs a simple ASCII character command set that allows complete
control of the instrument.

Long commands and arguments (longer than two characters) are separated by a
space character (0x20), and each long command string must be terminated with
a carriage return character (x0D). Long commands do not echo until the 0x0D
character is received.

Short commands (2 characters or less) are preceded with a null character
(0x00), which is not echoed, but each command character is echoed as it is
sent.

<0x00>command<(0x20)<argument1>(0x20)<agrument2>(0x0D)>

For example, the command "\0A1" generates the following response: "A1 2450"

Commands:
"\0A1"

      Returns device name: "2450"
"chn 0 5120\r" Enable analog channel 0 to measure an N type TC as the first scan list member
"chn 1 514\r"  Enable analog channel 2 to measure +/-100 mV as the second scan list member
"chn 2 3331\r" Enable analog channel 3 to measure +/-1 V as the third scan list member
"xrate 1871 10\r" Burst rate is 10 Hz,
               sampling frequency SF=79, averaging frequency AF=7
               SF+AF*256 = 79+7*256 = 1971, burst rate B = 8000/((SF+1)*(AF+3))
"\0S1"         Start the scanning processes, causes the DI-245 to respond with a continuous binary
               stream of one 16-bit signed integers per enabled measurement.
               The stream sequence repeats until data acquisition is halted by the stop
               command.
"\0S0"         Stop the scanning processes

Valentyn Stadnytskyi,
October 2017 - July 2018
last update: May 29, 2019

"""

from numpy import concatenate,zeros,mean,std,uint16, nan
from serial import Serial
from time import time, sleep,gmtime, strftime
from sys import stdout
import os.path
from pdb import pm
from struct import unpack as struct_unpack

import traceback

import logging
from logging import error,warning,info,debug


__version__ = '2.0.2' #

class Driver(object):

    def __init__(self, serial_number = None):
        """        instance init command
        """
        self.port = None
        self.timeout = 2
        self.acquiring = False
        #self.serial_number = '56671FE4A'


    def init(self):
        """
        orderly initialization of the DI-245 driver object

        Parameters
        ----------

        Returns
        -------

        Examples
        --------
        >>> driver.init()
        """

        if len(self.available_ports) != 0:
            self.port = self.use_com_port()
            self.stop_scan()
            self.description = {}
            self.description['Device name'] = self.query(command=b'A1')[2:]
            self.description['Firmware version'] = self.query(command=b'A2')[2:]
            self.description['Last Calibration date in hex'] = self.query(command=b'A7')[2:]
            self.description['Serial Number'] = self.query(command=b'NZ')[2:]
            for i in self.description.keys():
                info("{},{}".format(i, self.description[i]))
            info('Complete: Initialization of the DI-245 with SN {}'.format(self.description['Serial Number']))
        else:
            info('no DI-245 available')

    def use_com_port(self,serial_number = None):
        """
        1) connect to the serial port in self.available_ports(N)
        2) stops scanning if one is in progress
        3) tries to set buffer size

        property objects that return a list of com ports that have DI245 in the description.

        Parameters
        ----------
        serial_number :: str , optional
            serial number of the device as a string. If left blank the first avaialable DI245 will be selected.

        Returns
        -------
        port :: ''serial.serialposix.Serial''
            pyserial port object

        Examples
        --------
        >>> port = driver.use_com_port(serial_number = '56671FE4A')
        >>> port
        Serial<id=0x3e18c30, open=True>(port='COM23', baudrate=115200, bytesize=8, parity='N', stopbits=1, timeout=0.1, xonxoff=False, rtscts=True, dsrdtr=False)
        """
        port_name = None
        if len(self.available_ports) != 0:
            if serial_number is not None:
                import serial.tools.list_ports
                devices = serial.tools.list_ports.comports()
                for device in devices:
                    if device.serial_number == serial_number:
                        port_name = device.device
            else:
                port_name = self.available_ports[0]
        if port_name is not None:
            port = Serial(port_name, baudrate=115200, rtscts=True, timeout=0.1)
            #self.stop_scan()
            port.flushInput()
            port.flushOutput()
            port.set_buffer_size(rx_size = 409200)
        else:
            port = None
        return port

    @property
    def available_ports(self):
        """
        property objects that return a list of com ports that have DI245 in the description.

        Parameters
        ----------

        Returns
        -------
        list :: list
            list of com ports

        Examples
        --------
        >>> driver.available_ports
        ['COM23']
        """
        import serial.tools.list_ports
        lst = serial.tools.list_ports.comports()
        available_ports = []
        debug('looking for DI245 available ....')
        for element in lst:
            debug('checking %r' % element.device)
            if element.description.find('DI245') > -1:
                debug("the DI-245 is available at %r" %(element.device))
                available_ports.append(element.device)
        return available_ports

    def read(self, Nbytes = None, port = None, timeout = 10):
        """
        read from serial port buffer

        Parameters
        ----------
        Nbytes :: integer, optpional
            specify how many bytes to read. the default value makes this command behave as readline, will read entire buffer.

        port :: ''serial.serialposix.Serial'', optional
            pyserial port object. if left default, port will become self.port from the driver class.
        timeout :: float


        Returns
        -------
        string :: str
            string representation of read data from serial buffer

        Examples
        --------
        >>> driver.read()
        """
        from time import time, sleep
        tstart = time()
        sleep(timeout/10.)
        buff = 'timeout'
        if port is None:
            port = self.port
        if Nbytes is None:
            buff = port.readline()
        else:
            while time() - tstart < timeout:
                debug('while loop %r %r' % (time(),1))
                if self.waiting[0] >= Nbytes:
                    buff = port.read(Nbytes)
                    break
                else:
                    sleep(timeout)
        return buff

    def write(self,command, port = None):
        """
        write into serial port buffer

        Parameters
        ----------
        command :: string
            input command written in serial port input buffer
        port :: ''serial.serialposix.Serial''
            pyserial port object


        Returns
        -------
        flag :: boolean
            boolean

        Examples
        --------
        >>> driver.write(b'S0')
        """
        if port is None:
            port = self.port
        try:
            if port.isOpen():
                port.flushInput()
                port.flushOutput()
                port.write(command)
                result = True
            else:
                result = False
        except:
            error(traceback.format_exc())
            result = False
        return result

    def query(self,command, port = None, Nbytes = None):
        """
        query is a write-read command

        Parameters
        ----------
        command :: string
            input command written in serial port input buffer
        port :: ''serial.serialposix.Serial''
            pyserial port object
        Nbytes :: integer
            number of bytes expected as a result of command execution

        Returns
        -------
        string :: str
            response string

        Examples
        --------
        >>> query.
        """
        if port is None:
            port = self.port
        self.write(command = command, port = port)
        response = self.read(Nbytes = Nbytes, port =port)
        return response

    def flush(self,port=None, input = True, output = True):
        """
        flushes input and output buffers of a given port

        Parameters
        ----------
        port :: ''serial.serialposix.Serial''
            pyserial port object, default is None. If None takes self.port
        input :: flag, optional
            boolean flag whether to flush input port. default is True
        output :: flag, optional
            boolean flag whether to flush output port. default is True

        Returns
        -------

        Examples
        --------
        >>> driver.flush(port)
        """
        if port is None:
            port = self.port
        port.flushInput()
        port.flushOutput()


    def close(self, port = None):
        """
        orderly closes serial port associated with the class

        Parameters
        ----------
        port :: ''serial.serialposix.Serial'', optional
            pyserial port object, if left blank the self.port is assumed.

        Returns
        -------

        Examples
        --------
        >>> driver.close()
        """
        if port is None:
            port = self.port
        try:
            if port.isOpen():
                port.close()
                result = True
            else:
                result = False
        except:
            error(traceback.format_exc())
            result = False
        return result

    #Scanning\data acquisition section
    #an example: chn(0x20)member(0x20)value(0x0D)
    #2-byte value needs to be converted from binary to int. The binary 2 byte start counting from right.
    #The values in the function definition are default values in case user does not specify them.
    def config_channels(self,scan_lst = ['0','1','2','3'],phys_ch_lst = ['0','1','2','3'],gain_lst = ['5','5','5','T-thrmc'], rate = 0):
        """
        configures channels: maps physical channel list on the scan list with defined gains.
        configures readout rate.

        Parameters
        ----------
        scan_lst :: list
            scan order list

        phys_ch_lst :: list
            physical channel order list

        gain_lst :: list
            list of gains

        rate :: float
            rate of data collection

        Returns
        -------
        ToDo... Add what is returned if any.

        Examples
        --------
        >>> driver.config_channels
        """

        self.scan_lst = scan_lst
        self.phys_ch_lst = phys_ch_lst
        self.gain_lst = gain_lst

        _config_dict_gain = {}
        _config_dict_gain['0.010'] = '00101'
        _config_dict_gain['0.025'] = '00100'
        _config_dict_gain['0.05'] = '00011'
        _config_dict_gain['0.1'] = '00010'
        _config_dict_gain['0.25'] = '00001' #works
        _config_dict_gain['0.5'] = '00000'
        _config_dict_gain['1'] = '01101'
        _config_dict_gain['2.5'] = '01100'
        _config_dict_gain['5'] = '01011' #
        _config_dict_gain['10'] = '01010'
        _config_dict_gain['25'] = '01001'
        _config_dict_gain['50'] = '01000'
        _config_dict_gain['B-thrmc'] = b'10000'
        _config_dict_gain['E-thrmc'] = b'10001'
        _config_dict_gain['J-thrmc'] = b'10010'
        _config_dict_gain['K-thrmc'] = b'10011'
        _config_dict_gain['N-thrmc'] = '10100'
        _config_dict_gain['R-thrmc'] = '10101'
        _config_dict_gain['S-thrmc'] = '10110'
        _config_dict_gain['T-thrmc'] = '10111'
        result = []
        for i in range(len(self.scan_lst)):
            config_byte = str(int('000'+_config_dict_gain[self.gain_lst[i]]+'0000' +
                                  bin(int(self.phys_ch_lst[i]))[2:].zfill(4),2))
            ch_config_command = b'chn '+bytes(self.scan_lst[i],'Latin-1')+b' '+bytes(config_byte,'Latin-1')+b' \x0D'

            command = ch_config_command
            Nbytes = len(command)
            debug('configuring: {}'.format(command))
            if self.query(command = command, Nbytes = Nbytes, port = self.port) == ch_config_command:
                result.append(True)
            else:
                result.append(False)

        xrate_config_command = b'xrate 4099 2000 \x0D'
        command = xrate_config_command
        Nbytes = len(command)
        debug('configuring: {}'.format(command))
        if self.query(command = command, Nbytes = Nbytes, port = self.port) == xrate_config_command:
            result.append(True)
        else:
            result.append(False)
        return int(mean(result)), result

    def read_buffer(self, N_of_channels, N_of_points = 1):
        """
        break down read_number function into two steps
        1) read buffer (this function)
        2) convert data

        reads data from output serial buffer

        Parameters
        ----------
        N_of_channels :: integer
            number of channels to read
        N_of_points :: integer, optional
            number of channels to datapoints to read, default value is 1

        Returns
        -------
        buffer :: bytes string
            raw data from the serial output buffer

        Examples
        --------
        >>> raw_data = driver.read_number(N_of_channels = 4, N_of_points = 2)
        """
        data_bytes = self.port.read(2*N_of_channels*N_of_points)
        return data_bytes

    def convert_buffer_to_array(self, buffer, N_of_channels, N_of_points = 1):
        """
        break down read_number function into two steps
        1) read buffer
        2) convert data (this function)


        convert buffer data into array

        Parameters
        ----------
        buffer :: bytes string
            raw data from the serial output buffer
        N_of_channels :: integer
            number of channels to read
        N_of_points :: integer, optional
            number of channels to datapoints to read, default value is 1

        Returns
        -------
        array :: numpy.ndarray
            numpy array



        Examples
        --------
        >>> arr = driver.read_number(buffer = raw_data, N_of_channels = 4, N_of_points = 2)

        """

        raise NotImplementedError

    def sync_read_buffer(self,N_of_channels = 4):
        from struct import unpack
        syncronizing = True
        while syncronizing:
            read_byte_temp = self.port.read(2)
            read_byte = bin(unpack("H", read_byte_temp)[0])[2:].zfill(16)
            sync_byte = read_byte[15]
            if sync_byte == 0:
                read_byte_temp = self.port.read(N_of_channels*2-2)
            else:
                syncronizing = False


    def read_number(self, N_of_channels, N_of_points = 1):
        """
        reads N channels(N_of_channels) with N points(N_of_points)
        and puts them in an array (N channels x N points)

        Parameters
        ----------
        N_of_channels :: integer
            number of channels to read
        N_of_points :: integer, optional
            number of channels to datapoints to read, default value is 1

        Returns
        -------
        array :: numpy.ndarray
            numpy array

        Examples
        --------
        >>> arr = driver.read_number(N_of_channels = 4, N_of_points = 2)
        >>> arr.shape
        (4,2)
        """
        from struct import unpack
        channels_to_read = N_of_channels
        datapoints_to_read = N_of_points
        value_array = zeros((channels_to_read,N_of_points),dtype = 'int16')
        for k in range(N_of_points):
            for j in range(channels_to_read):
                #value_array[2*j] = time.time()
                tempt_t = time()
                read_byte_temp = self.port.read(2)
                try:
                    read_byte = bin(unpack("H", read_byte_temp)[0])[2:].zfill(16)
                except Exception as e:
                    error('read_byte = %r and error %r' % (read_byte_temp,e))
                read_byte_lst = list(read_byte)
                sync_byte = read_byte_lst[15] #this is the byte 0 that is issued in DI-245 for sync. 0 stand for the beginning of channel(s) data stream. Hence, every set of readouts  starts with 0.
                del(read_byte_lst[15]) #this needs to be used
                del(read_byte_lst[7])  #this needs to be used
                read_byte = ""
                for i in read_byte_lst:
                    read_byte += str(i)
                int_val = int(read_byte,2)
                value_array[j,k] = int_val
        return value_array

    @property
    def waiting(self, port = None):
        """
        returns number of bytes waiting in the serail buffer
        (in, out)

        Parameters
        ----------
        port :: optional
            serial port object

        Returns
        -------
        waiting :: tuple
            tuple representaiotn of number of bytes waiting in input and output buffers

        Examples
        --------
        >>> driver.waiting()
        (0,0)
        """
        if port is None:
            port = self.port
        try:
            result = (port.inWaiting(),port.out_waiting)
        except Exception as err:
            error(err)
            result = (nan,nan)
        return result
    #this method sends a proper command to start the scan.
    def start_scan(self):
        """
        starts data acquisition. issues start command "S1" that initializes data stream from the DI 245

        Parameters
        ----------

        Returns
        -------

        Examples
        --------
        >>> driver.start_scan()
        """
        self.flush()
        self.write(b'(0x00) S1')
        self.acquiring = True
        self.sync_read_buffer()
        info('The configured measurement(s) has(have) started')

    def stop_scan(self):
        """
        stop current data acquisition. issues start command "S0" that initializes data stream from the DI 245

        Parameters
        ----------

        Returns
        -------

        Examples
        --------
        >>> driver.stop_scan()
        """
        if self.port.isOpen():
            self.write(b'S0')
            result = False
        else:
            result = None
        self.acquiring = result
        self.flush()


    def stop(self):
        """
        stop current data acquisition. issues start command "S0" that initializes data stream from the DI 245

        Parameters
        ----------

        Returns
        -------

        Examples
        --------
        >>> driver.stop_scan()
        """
        debug('full stop command executed')
        self.stop_scan()
        self.close()

    def kill(self):
        """
        orderly stop and deletion of the object

        Parameters
        ----------

        Returns
        -------

        Examples
        --------
        >>> driver.stop_scan()
        """
        debug('kill')
        self.stop()
        del self


if __name__ == "__main__":
    from tempfile import gettempdir

    logging.basicConfig(#filename = gettempdir()+'/di_245_driver.log',
                        level=logging.DEBUG,
                        format="%(asctime)s %(levelname)s: %(message)s")
    driver = Driver()
    self = driver

    print('----- The driver for the DI-245 -----')
    print('*self*  is already created instance')
