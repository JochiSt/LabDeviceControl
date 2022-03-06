"""
    script for reading the PeakTech DMM 3315

    inspired by pyUSBtin
"""

import serial
import sys
from time import sleep
import threading

PY_VERSION = sys.version_info[0]

class PeakTech_DMM3315_Exception(Exception):
    def __init__(self, *args, **kwargs):
        super().__init__(self, *args, **kwargs)

# this class represents a can message
class PeakTech_DMM3315(object):
    """ enums for rx thread """
    RX_THREAD_STOPPED, RX_THREAD_RUNNING, RX_THREAD_TERMINATE = range(3)

    """ timeout for reading from serial port """
    READ_TIMEOUT = 1

    def __init__(self):
        """ initialiser """
        self.serial_port = None
        self.rx_thread_state = PeakTech_DMM3315.RX_THREAD_STOPPED
        self.listeners = []

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
            self.serial_port = serial.Serial(port, baudrate=2400,
                                             bytesize=serial.SEVENBITS,
                                             parity=serial.PARITY_ODD,
                                             stopbits=serial.STOPBITS_ONE,
                                             timeout=PeakTech_DMM3315.READ_TIMEOUT)

            self.serial_port.setDTR(1)    # switch on DTR, otherwise it will not work

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

    def open_can_channel(self, baudrate, mode):
        """Open CAN channel.
           Set given baudrate and open the CAN channel in given mode.

           Keyword arguments:
            baudrate -- Baudrate in bits/second
            mode -- CAN bus accessing mode
        """


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

    def rx_thread(self):
        """ main rx thread. this thread will take care to
            handle the data from the serial port"""

        print("rx thread started")

        """ process data as long as requested """
        while self.rx_thread_state == PeakTech_DMM3315.RX_THREAD_RUNNING:

            # fetch bytes if available
            rxcount = self.serial_port.inWaiting()
            if rxcount > 0:
                # fetch all data from serial buffer
                buf = self.serial_port.read(rxcount)
                if PY_VERSION == 2:
                    buf = [ord(b) for b in buf]

                print(buf)

                # create value and unit form read string
                value, unit = None, None
                # give these information to the listeners
                for listener in self.listeners:
                    listener(value, unit)

                # clear message
                self.incoming_message = ""

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
