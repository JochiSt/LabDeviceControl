# -*- coding: utf-8 -*-

import time
from PeakTech_DMM3315 import PeakTech_DMM3315

mm = PeakTech_DMM3315()

raw = b'001996002\r\n'
print(raw)
print( mm.parseString(raw) )

#mm.connect("COM9")
#time.sleep(5)
#mm.disconnect()

