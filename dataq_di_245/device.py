# encoding=utf8
"""
Hybrid EPICS IOC / direct TCP server for DATAQ 245 device.





Four-channel USB Voltage and Thermocouple DAQ,
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

For example, the command "\0A1" generates the following response: "A12450"

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

Valentyn Stadnytskyi Nov 2017
"""

from numpy import concatenate,zeros,mean,std,uint16,int16, nan
from time import time, sleep, gmtime, strftime
from sys import stdout
import os.path
from pdb import pm
from logging import error,warn,info,debug

from dataq_di_245.driver import Driver
from ubcs_auxiliary.saved_property import DataBase, SavedProperty
from ubcs_auxiliary.threading import new_thread

import msgpack
import msgpack_numpy as m

class Device(object):
    db = DataBase(root = 'TEMP', name = 'dataq_covid19')
    name = SavedProperty(db,'name', 'dataq_covid19').init()
    prefix = SavedProperty(db,'prefix', 'NIH:DI245').init()
    scan_lst = SavedProperty(db,'scan_lst', ['0','1','2','3']).init()
    phys_ch_lst = SavedProperty(db,'phys_ch_lst', ['0','1','2','3']).init()
    gain_lst = SavedProperty(db,'gain_lst', ['5','5','5','5']).init()
    buffer_size = SavedProperty(db,'buffer_size', 4320000).init()
    packet_length = SavedProperty(db,'packet_length', 10).init()
    calib = SavedProperty(db,'calib',  [0.559, 2.7, 3.2, -1.2, 0]).init()
    time_out = SavedProperty(db,'time_out', 0).init()
    cjc_value = SavedProperty(db,'cjc_value', '').init()
    SN = SavedProperty(db,'SN', '').init()

    def __init__(self, name = None):
        if name is not None:
            self.name = name
        else:
            self.name = 'DI245_noname'
        self.recording_flag = False


    def init(self, serial_number):
        self.dev = Driver()
        self.driver = self.dev
        success = self.driver.init(serial_number)

        if success:
            self.configure_device()
            debug('DI245 is found: %r' % self.driver.available_ports)
            self.info_dict = {}
            self.info_dict['scan_lst'] = self.scan_lst
            self.info_dict['phys_ch_lst'] = self.phys_ch_lst
            self.info_dict['gain_lst'] = self.gain_lst
            self.info_dict['RingBuffer_size'] = self.buffer_size
            self.info_dict['calib'] = self.calib
            self.info_dict['time_out'] = self.time_out
            self.info_dict['cjc_value'] = self.cjc_value
            self.info_dict['SN'] = self.SN
            reply = True
        else:
            reply = False
            error('DI-245 is not found')

        return reply

    def stop(self):
        self.full_stop()


    def configure_device(self):
        self.driver.stop_scan()
        from circular_buffer_numpy.circular_buffer import CircularBuffer
        self.buffer = CircularBuffer(shape = (self.buffer_size,len(self.scan_lst)), dtype = 'int16')#4320000
        self.buffer.packet_length = self.packet_length
        print(self.scan_lst,self.phys_ch_lst,self.gain_lst)
        self.dev.config_channels(scan_lst=self.scan_lst,phys_ch_lst=self.phys_ch_lst,gain_lst = self.gain_lst)


    def run_once(self):
        """
        """
        from tempfile import gettempdir
        from numpy import mean
        from time import time
        root = gettempdir()
        length = self.packet_length
        if self.driver.waiting[0] > length*4*2:
            value_array = self.dev.read_number(N_of_channels = len(self.scan_lst), N_of_points = length).T - 8192
            self.buffer.append(value_array)
            T_top = mean(value_array[:,2])*0.036621+100
            T_bottom = mean(value_array[:,3])*0.036621+100
            Vs = mean(value_array[:,0]*10.0/(2**13))
            Vo = mean(value_array[:,1]*5.0/(2**13))
            rh = mean(self.relative_humidity(Vs,Vo,T_top,T_bottom))
            if self.recording_flag:
                with open(root + '/covid19_DI245.txt',"a") as f:
                    string = f'{time()},{round(T_top,2)}, {round(T_bottom,2)}, {round(rh,2)}, {round(Vs,2)},{round(Vo,2)} \n'
                    f.write(string)
            from EPICS_CA.CAServer import casput
            casput('BigBox:TEMP_TOP',T_top)
            casput('BigBox:TEMP_BOTTOM',T_bottom)
            casput('BigBox:RH',rh)

        else:
            sleep(0.01)

    def run(self):
        while self.running:
            self.run_once()
        self.running = False

    def start(self, new_thread = True):
        from ubcs_auxiliary.threading import new_thread as thread
        from time import time
        self.driver.start_scan()
        self.time_start = self._time_start = time()
        self.running = True
        if new_thread:
            thread(self.run)
        else:
            self.run()

    def stop(self):
        from time import sleep
        self.running = False
        sleep(1)
        self.driver.stop_scan()

    def full_stop(self):
        try:
            self.dev.full_stop()
            self.running = False
        except:
            warn('dev is not initialized')

    def save_to_a_file(self):
        debug('save to a file pressed %r' % time())
        pass

    def run_full(self, serial_number = ''):
        self.init(serial_number = serial_number)
        self.start()

    def start_recording():
        self.recording_flag = True

    def stop_recording():
        self.recording_flag = False

    def relative_humidity(self,Vs,Vo,T1,T2):
        return 149.09*((Vo/Vs)-0.1515)/(1-0.002048*(0.5*(T1+T2)))


if __name__ == "__main__":
    from tempfile import gettempdir
    logging.basicConfig(filename=gettempdir()+'/di_245_DL.log',
                        level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")

"""
def relative_humidity(Vs,Vo,T1,T2):
    print(Vo[0],Vs[0],T1[0],T2[0])
    return 149.09*((Vo/Vs)-0.1515)/(1-0.002048*(0.5*(T1+T2)))

rh = relative_humidity(
device.buffer.get_last_N(2000)[:,0]*10.0/(2**13)
device.buffer.get_last_N(2000)[:,1]*5.0/(2**13)
device.buffer.get_last_N(2000)[:,2])*0.036621+100
device.buffer.get_last_N(2000)[:,3])*0.036621+100
Supply          output                T_top               T_bottom
5.126           1.95                  24.377              24.4142


"""
