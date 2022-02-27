#!/usr/bin/env python3

import time
from IMU import *

if __name__ == "__main__":
    xsens = Xsens(ShowError=False)     # initial the imu class

    # xsens.ConnectWithSerialNumber("DB5SGYLL")
    xsens.ConnectWithDeviceName("/dev/ttyUSB0")

    time.sleep(0.1)

    while(True):
        
        xsens.GetMeasure()

        if xsens.NewDataAvailable() == True:
            xsens.MarkDataOld()

            xsens.QuatToEuler ()
            # print(xsens.XsensTime*1e-4)
            s = ""
            s += str(time.time()) + " |Roll: %.2f" % (xsens.euler[0,0] * 180 / math.pi) + ", Pitch: %.2f" % (xsens.euler[0,1] * 180 / math.pi) + ", Yaw: %.2f " % (xsens.euler[0,2] * 180 / math.pi )
            print(s)
            # imu_data.write("%s\n" % s)   #store imu_data
        else:
            # print("No data")
            pass

