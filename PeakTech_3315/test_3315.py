# -*- coding: utf-8 -*-

import time
from PeakTech_DMM3315 import PeakTech_DMM3315

mm = PeakTech_DMM3315()
mm.connect("COM9")

time.sleep(10)

mm.disconnect()

