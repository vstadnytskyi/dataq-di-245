======
Driver
======

Start by importing dataq DI-245.

.. code-block:: python

    In [1]: from dataq_di_245 import Driver

    In [2]: driver = Driver()

    In [3]: driver.available_ports
    Out [3]: ['COM6']

    In [4]: driver.init()

    In [5]: driver.port
    Out [5]: Serial<id=0x2238bb0, open=True>(port='COM6', baudrate=115200, bytesize=8, parity='N', stopbits=1, timeout=0.1, xonxoff=False, rtscts=True, dsrdtr=False)

    In [6]: driver.config_channels(scan_lst = ['0','1','2','3'],phys_ch_lst = ['0','1','2','3'],gain_lst = ['0.5','0.5','0.5','0.5'], rate = 0)
    Out [6]: (1, [True, True, True, True, True])

    In [7]:  driver.start_scan()

    In [8]:  arr = driver.read_number(N_of_channels = 4, N_of_points = 10)

    In [9]: arr.shape, arr.mean()
    Out [9]: ((4, 10), 14335.75)

    In [10]: driver.waiting
    Out [10]: (17192, 0)

    In [11]: driver.acquiring
    Out [11]: True

    In [12]:  driver.description

    Out [12]: {'Device name': b'A12450', 'Firmware version': b'6B', 'Last Calibration date in hex': b'FFFFFFFF', 'Serial Number': b'XXXXXXXXXX'}



.. autoclass:: dataq_di_245.Driver
  :members:
