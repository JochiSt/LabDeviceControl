# -*- coding: utf-8 -*-

import time
from PeakTech_DMM3315 import PeakTech_DMM3315

def printMeasurement(value, unit):
    print(value, unit)

mm = PeakTech_DMM3315()

raw = b'001996002\r\n'
print(raw)
print( mm.parseString(raw) )

mm.connect("COM9")
mm.add_message_listener(printMeasurement)
time.sleep(5)
mm.disconnect()

