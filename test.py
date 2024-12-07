from api5 import pyDualArm
import time
import numpy as np

rbt = pyDualArm()
rbt.connect()
rbt.enableGravity()
rbt.loadUrdf()

rbt.enableRealTimeSim()
rbt.home()

rbt.move()

#rbt.storeData(True)

