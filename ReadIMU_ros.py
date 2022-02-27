#!/usr/bin/env python3

from IMU import *
import rospy
from geometry_msgs.msg import QuaternionStamped
import time
import threading

def IMU_reader(xsens):
    while not rospy.is_shutdown():
        xsens.GetMeasure()
        if xsens.NewDataAvailable() == True:
            xsens.MarkDataOld()

            xsens.QuatToEuler ()

            s = ""
            s += str(time.time()) + " |Roll: %.2f" % (xsens.euler[0,0] * 180 / math.pi) + ", Pitch: %.2f" % (xsens.euler[0,1] * 180 / math.pi) + ", Yaw: %.2f " % (xsens.euler[0,2] * 180 / math.pi )
            print(s)

def publisher(xsens):
    quat_pub = rospy.Publisher('Quaternion', QuaternionStamped, queue_size=1)
    rate = rospy.Rate(30) # 30hz

    while not rospy.is_shutdown():
        QuatStamped = QuaternionStamped()
        QuatStamped.header.stamp = rospy.Time.now()
        QuatStamped.quaternion.w = xsens.quat[0]
        QuatStamped.quaternion.x = xsens.quat[1]
        QuatStamped.quaternion.y = xsens.quat[2]
        QuatStamped.quaternion.z = xsens.quat[3]
        quat_pub.publish(QuatStamped)

        rate.sleep()

if __name__ == "__main__":

    rospy.init_node('XsensSDK', anonymous=True)

    use_serial_number = True
    serial_number = ""
    serial_port = ""

    if rospy.has_param('~serial_number'):
        serial_number = rospy.get_param('~serial_number')
    elif rospy.has_param('~serial_port'):
        serial_port = rospy.get_param('~serial_port')
        use_serial_number = False
    else:
        rospy.logfatal("Please provide either serial_number or serial_port")
        exit()

    xsens = Xsens(ShowError=False)     # initial the imu class

    connect = False
    if use_serial_number:
        connect = xsens.ConnectWithSerialNumber(serial_number)
    else:
        connect = xsens.ConnectWithDeviceName(serial_port)

    if connect:
        threading.Thread(target=IMU_reader, args=(xsens,)).start()

        try:
            publisher(xsens)
        except rospy.ROSInterruptException:
            pass
    else:
        rospy.logfatal("Connection Failed")