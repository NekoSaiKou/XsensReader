# XsensReader
Read Xsens, parse and publish to ROS

# Environment

* ROS1
* Python3

# Use without ROS

## Requirement
```python3 -m pip install -r requirements.txt```

### Run Example
Modify ReadIMU.py  

Connect Xsens with ```xsens.ConnectWithSerialNumber("SERIAL_NUMBER")``` with device SERIAL_NUMBER  
or  
Connect Xsens with ```xsens.ConnectWithDeviceName("SERIAL_PORT")``` with device port number

Run python script
```
python3 ReadIMU.py
```

### Get all available Xsens device port and serial number
```
python3 GetAvailableImu.py 
```

# Use with ROS

## Requirement
```python3 -m pip install -r requirements_ros.txt```

## Run node

After catkin_make and source setup.bash:

### Run with known Serial Number:
DB5SGYLL in this example

```
rosrun xsens_sdk ReadIMU_ros.py _serial_number:=DB5SGYLL
```

### Run with known Serial Port:
DB5SGYLL in this example

```
rosrun xsens_sdk ReadIMU_ros.py _serial_port:="/dev/ttyUSB0"
```

If both _serial_port and _serial_number are given, _serial_port option will be ignored.

### Get all available Xsens device port and serial number

```
rosrun xsens_sdk GetAvailableImu.py 
```

### Publish Topic

* geometry_msgs/QuaternionStamped