"""
    script for reading the PeakTech DMM 3315

    inspired by pyUSBtin
"""

import serial
from time import sleep
import threading

class PeakTech_DMM3315_Exception(Exception):
    def __init__(self, *args, **kwargs):
        super().__init__(self, *args, **kwargs)

# this class represents a can message
class PeakTech_DMM3315(object):
    """ enums for rx thread """
    RX_THREAD_STOPPED, RX_THREAD_RUNNING, RX_THREAD_TERMINATE = range(3)

    """ timeout for reading from serial port """
    READ_TIMEOUT = 1

    """ enumerate all functions """
    VOLTAGE, uACURRENT, mACURRENT, ACURRENT, OHM, CONTINUITY, DIODE, FREQUENCY, CAPACITY, TEMPERATURE, ADP0, ADP1, ADP2, ADP3 = range(14)

    """ function list """
    FUNCTION_TABLE = {
            0x3B : (VOLTAGE,        "V"),
            0x3D : (uACURRENT,    "A"),
            0x39 : (mACURRENT,    "A"),
            0x3F : (ACURRENT,       "A"),
            0x33 : (OHM,                "Ohm"),
            0x35 : (CONTINUITY,     "Ohm"),
            0x31 : (DIODE,              "V" ),
            0x32 : (FREQUENCY,      "Hz"),
            0x36 : (CAPACITY,         "F" ),
            0x34 : (TEMPERATURE,  "C"),
            0x3E : (ADP0,""),
            0x3C : (ADP1,""),
            0x38 : (ADP2,""),
            0x3A : (ADP3,""),
    }

    RANGE_TABLE = [
            # VOLTAGE
            [400.0, 4.000, 40.00, 400.0, 4000],
            # uA CURRENT
            [40.00, 400.0],
            # mA CURRENT
            [400.0, 4000],
            # A Current
            [40],
            # OHM
            [400.0, 4.000e3, 40.00e3, 400.0e3, 4.00e6, 40.0e6],
            # CONTINUITY
            [40],
            # DIODE
            [40],
            # FREQUENCY
            [4.000e3, 40.00e3, 400.0e3, 4.000e3, 40.00e3, 400.0e3],
            #[40.00e3, 400.0e3, 4.000e3, 40e3, 400e3, 4000e3],
            # CAPACITY
            [4e-9, 40e-9, 400e-9, 4e-6, 40e-6, 400e-6, 4e-3, 40e-3],
            # TEMPERATURE
            [40],
            ]

    def __init__(self):
        """ initialiser """
        self.serial_port = None
        self.rx_thread_state = PeakTech_DMM3315.RX_THREAD_STOPPED
        self.listeners = []
        self.DEBUG = False      # set to True to debug the received messages

    def connect(self, port):
        """Connect to PeakTech_DMM3315 on given port.
           Opens the serial port, clears pending characters and send close command
           to make sure that we are in configuration mode.

           Keyword arguments:
            port -- name of serial port

           Throws:
            PeakTech_DMM3315_Exception in case something goes wrong
        """
        try:
            # open serial port
            self.serial_port = serial.Serial(port,
                                             baudrate=2400,
                                             bytesize = serial.SEVENBITS,
                                             parity = serial.PARITY_ODD,
                                             stopbits = serial.STOPBITS_ONE,
                                             timeout = PeakTech_DMM3315.READ_TIMEOUT)

            self.serial_port.dtr = True    # switch on DTR, otherwise it will not work
            self.serial_port.rts = False
            # clear input and output
            self.serial_port.flush()
            self.serial_port.flushInput()  # reset_input_buffer()

            # some debug info
            print("connected to PeakTech_DMM3315")

            try:
                # start rx thread:
                self.start_rx_thread()
            except serial.SerialTimeoutException as e:
                raise PeakTech_DMM3315_Exception(e)

        except Exception as e:
            self.serial_port.close()
            raise e
        #except serial.SerialException as e:
        #   raise PeakTech_DMM3315_Exception("{0} - {1}: {2}".format(port, e.errno, e.strerror))

    def disconnect(self):
        """Disconnect. Close serial port connection"""
        try:
            self.stop_rx_thread()
            self.serial_port.close()
        except serial.SerialException as e:
            raise PeakTech_DMM3315_Exception("{0} - {1}: {2}".format(self.serial_port.name, e.errno, e.strerror))

    def start_rx_thread(self):
        """ start the serial receive thread"""
        self.rx_thread_state = PeakTech_DMM3315.RX_THREAD_RUNNING
        thread = threading.Thread(target=self.rx_thread, args=())
        thread.daemon = True
        thread.start()
        #start_new_thread(self.rx_thread, (self, self.serial_port))

    def stop_rx_thread(self):
        """ stop the serial receive thread.
            note: this will block until the thread was shut down"""
        if self.rx_thread_state == PeakTech_DMM3315.RX_THREAD_STOPPED:
            # already stopped, thus return
            return

        # tell the thread to exit
        self.rx_thread_state = PeakTech_DMM3315.RX_THREAD_TERMINATE
        while self.rx_thread_state != PeakTech_DMM3315.RX_THREAD_STOPPED:
            # wait for thread to end, sleep 1ms
            sleep(0.001)

    def parseString(self, raw):
        if len(raw) != 11:
            print("input data length mismatch %d received 11 exspected"%(len(raw)))
            return None, None

        if self.DEBUG:
            print("raw", raw)
            for r in raw:
                print("%x"%(int(r)), end=" ")
            print()

        MMrange = int(raw[0])
        MMrange -= 0x30
        MMdigits = int(raw[1:5])    # convert into digits
        MMfunct = int(raw[5])
        try:
            function, unit = PeakTech_DMM3315.FUNCTION_TABLE[MMfunct]
        except IndexError:
            print("FUNCTION_TABLE[",MMfunct, "] not found")
            function = 0
            unit = None
        try:
            multiplier = PeakTech_DMM3315.RANGE_TABLE[function][MMrange]
        except IndexError:
            print("RANGE_TABLE[",function, "][", MMrange, "] not found")
            multiplier = 4000   # results in digits = display

        multiplier = multiplier / 4000
        value = MMdigits * multiplier
        return value, unit

    def rx_thread(self):
        """ main rx thread. this thread will take care to
            handle the data from the serial port"""

        print("rx thread started")

        """ process data as long as requested """
        while self.rx_thread_state == PeakTech_DMM3315.RX_THREAD_RUNNING:
            raw = self.serial_port.readline()
            # create value and unit form read string
            value, unit = self.parseString(raw)
            # give these information to the listeners
            for listener in self.listeners:
                listener(value, unit)
            # clear message
            raw = ""
        # thread stopped...
        self.rx_thread_state = PeakTech_DMM3315.RX_THREAD_STOPPED

    def add_message_listener(self, func):
        """ add a message listener (callback)"""
        self.listeners.append(func)

    def remove_message_listener(self, func):
        """ remove message listemer"""
        if func in self.listeners:
            self.listeners.remove(func)
        else:
            raise PeakTech_DMM3315_Exception("ERROR: failed to remove listener")
