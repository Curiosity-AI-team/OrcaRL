#!/usr/bin/env python3
import math

import time
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

G = 9.80665
MagFieldConversion_uT_T = 0.000001

mpu = MPU9250(
    address_ak=AK8963_ADDRESS,
    address_mpu_master=MPU9050_ADDRESS_68, # In 0x68 Address
    address_mpu_slave=None,
    bus=1,
    gfs=GFS_1000,
    afs=AFS_8G,
    mfs=AK8963_BIT_16,
    mode=AK8963_MODE_C100HZ)

# Apply the settings to the registers and calibrat3
mpu.calibrate()
mpu.calibrateAK8963()
mpu.configure() 