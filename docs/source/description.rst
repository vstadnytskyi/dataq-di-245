===========
Description
===========

The driver, device and Channel Access EPICS input-output controller for DI-245 Thermocouple and Voltage Data Acquisition System by DATAQ Instruments. The link to the `DI-245 manual <https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=1&ved=2ahUKEwj-orv8lMzlAhUBnFkKHa2fAFsQFjAAegQIARAC&url=https%3A%2F%2Fwww.dataq.com%2Fresources%2Fpdfs%2Fmanuals%2Fdi-245-manual.pdf&usg=AOvVaw3O8UGfb5zWhoJfyNTBja4l>`_  and the `website <https://www.dataq.com/products/di-245/>`_ .

********
Driver
********


Four-channel USB Voltage and Thermocouple DAQ driver,
Resolution:  14-bit 
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
"xrate 1871 10\r" Burst rate is 10 Hz, sampling frequency SF=79, averaging frequency AF=7, SF+AF*256 = 79+7*256 = 1971, burst rate B = 8000/((SF+1)*(AF+3))

"\0S1"         Start the scanning processes, causes the DI-245 to respond with a continuous binary stream of one 16-bit signed integers per enabled measurement. The stream sequence repeats until data acquisition is halted by the stop command.
"\0S0"         Stop the scanning processes


Start by importing dataq DI-245.

.. code-block:: python

    import dataq_di_245
