#!/usr/bin/env python3

from IMU import *
import rospy
from geometry_msgs.msg import QuaternionStamped
from sensor_msgs.msg import Imu as ImuMSG
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

def publisher(xsens, quat_topic, imu_topic, imu_frame, publish_rate):
    quat_pub = rospy.Publisher(quat_topic, QuaternionStamped, queue_size=1)
    imu_pub = rospy.Publisher(imu_topic, ImuMSG, queue_size=1)
    rate = rospy.Rate(publish_rate) # 30hz

    while not rospy.is_shutdown():
        x, y, z, w = xsens.GetQuat()
        # Publish Quat
        QuatStamped = QuaternionStamped()
        QuatStamped.header.stamp = rospy.Time.now()
        QuatStamped.quaternion.w = w
        QuatStamped.quaternion.x = x
        QuatStamped.quaternion.y = y
        QuatStamped.quaternion.z = z
        quat_pub.publish(QuatStamped)

        # Publish IMU
        IMU_msg = ImuMSG()
        IMU_msg.header.stamp = rospy.Time.now()
        IMU_msg.header.frame_id = imu_frame
        IMU_msg.orientation.w = w
        IMU_msg.orientation.x = x
        IMU_msg.orientation.y = y
        IMU_msg.orientation.z = z
        IMU_msg.orientation_covariance = (-1., )*9
        IMU_msg.angular_velocity_covariance = (-1., )*9
        IMU_msg.linear_acceleration_covariance = (-1., )*9
        imu_pub.publish(IMU_msg)

        rate.sleep()

if __name__ == "__main__":

    rospy.init_node('XsensSDK', anonymous=True)

    serial_number = ""
    serial_port = ""
    quat_topic = "/Quaternion"
    imu_topic = "/IMU"
    imu_frame = "IMU"
    publish_rate = 30

    use_serial_number = False
    use_serial_port = False
    if rospy.has_param('~serial_number'):
        serial_number = rospy.get_param('~serial_number')
        if not serial_number == "disable":
            use_serial_number = True

    if rospy.has_param('~serial_port'):
        serial_port = rospy.get_param('~serial_port')
        if not serial_port == "disable":
            use_serial_port = True
    
    if not use_serial_number and not use_serial_port:
        rospy.logfatal("Please provide either serial_number or serial_port")
        exit()
    
    if use_serial_number and use_serial_port:
        rospy.loginfo("Both Serial Number and Serial Port are provided, use Serial Number first")

    if rospy.has_param('~quat_topic'):
        quat_topic = rospy.get_param('~quat_topic')
    
    if rospy.has_param('~imu_topic'):
        imu_topic = rospy.get_param('~imu_topic')  

    if rospy.has_param('~imu_frame'):
        imu_frame = rospy.get_param('~imu_frame') 

    if rospy.has_param('~publish_rate'):
        publish_rate = rospy.get_param('~publish_rate') 

    xsens = Xsens(ShowError=False)     # initial the imu class

    connect = False
    if use_serial_number:
        rospy.loginfo("Try Connect to device using serial number")
        connect = xsens.ConnectWithSerialNumber(serial_number)
    else:
        rospy.loginfo("Try Connect to device using serial port")
        connect = xsens.ConnectWithDeviceName(serial_port)

    if connect:
        threading.Thread(target=IMU_reader, args=(xsens,)).start()

        try:
            publisher(xsens, quat_topic, imu_topic, imu_frame, publish_rate)
        except rospy.ROSInterruptException:
            pass
    else:
        rospy.logfatal("Connection Failed")