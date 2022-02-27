
import math
import numpy
import serial
from serial.tools import list_ports
import struct
import threading

#set Xsens class
class Xsens:
    def __init__(self, ShowError=False):
        self.ShowError = ShowError

        # Serial Port Setting
        self.serial_port = None
        self.baudrate = 115200
        self.device_name = ""
        self.device_serial_number = ""

        self.newData = False

        # Data Processing
        self.quat = numpy.zeros(shape=(4), dtype='float_')
        self.quat_temp = ['']*4

        self.euler = numpy.zeros(shape=(1,3), dtype='float_')

        self.len = 5    # Data length without MTData2 data field (Preamble, BID, MTData, LEN and Checksum)
        self.receivedBytes = ['']*self.len

        # Threading
        self.data_lock = threading.Lock()
        self.new_data_lock = threading.Lock()

        # MTData2 content
        self.startbyte = b'\xfa'                           #set the first check point
        self.secondbyte = b'\xff'                          #set the second check point
        self.mtdata2 = b'\x36'
        self.TypeHex = [b'\x08',b'\x10',b'\x20',b'\x30',b'\x40',b'\x50',b'\x70',b'\x80',b'\xA0',b'\xC0',b'\xD0',b'\xE0']
        self.MTData2StartIDX = 4

    def ConnectWithDeviceName(self, target_port):
        device_list = list_ports.comports()
        connect = False
        for device in device_list:
            if device.manufacturer != "Xsens" or device.product != "Motion Tracker Dev. Board" or device.device != target_port:
                continue
            else:
                try:
                    self.serial_port = serial.Serial(
                        port=target_port, 
                        baudrate=self.baudrate, 
                        bytesize=serial.EIGHTBITS,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE)
                    connect = True
                    self.device_name = device.device
                    self.device_serial_number = device.serial_number
                    print("Connect to ", device.device, " Serial Number: ", device.serial_number)
                except (ValueError, serial.SerialException) as e:
                    print("[IMU] Connect to ", target_port, " Error Occurred!!!")
                    print(e)
                    connect = False
                break

        if not connect:
            print("[IMU] Connect to ", target_port, " Failed!!!")

        return connect

    def ConnectWithSerialNumber(self, serial_number):
        device_list = list_ports.comports()
        connect = False
        for device in device_list:
            if device.manufacturer != "Xsens" or device.product != "Motion Tracker Dev. Board" or device.serial_number != serial_number:
                continue
            else:
                try:
                    target_port = device.device
                    self.serial_port = serial.Serial(
                        port=target_port, 
                        baudrate=self.baudrate, 
                        bytesize=serial.EIGHTBITS,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE)
                    connect = True
                    self.device_name = device.device
                    self.device_serial_number = device.serial_number
                    print("Connect to ", device.device, " Serial Number: ", device.serial_number)
                except (ValueError, serial.SerialException) as e:
                    print("[IMU] Connect to ", target_port, " Error Occurred!!!")
                    print(e)
                    connect = False
                break

        if not connect:
            print("[IMU] Connect to device: ", serial_number, " Failed!!!")

        return connect

    def NewDataAvailable(self):
        ##
        # If new data is available

        self.new_data_lock.acquire()
        new_data_available = self.newData
        self.new_data_lock.release()
        return new_data_available

    def MarkDataOld(self):
        ##
        # Mark Data as Old, for user

        self.new_data_lock.acquire()
        self.newData = False
        self.new_data_lock.release()

    def MarkDataNew(self):
        ##
        # Mark Data as New if a complete MTData2 is received

        self.new_data_lock.acquire()
        self.newData = True
        self.new_data_lock.release()

    def GetMeasure(self):
        ##
        # Read until a complete MTData2 is gotten from serial port or Return if buffer empty
        # This function should be called frequently to prevent receiving too OLD data

        next_idx = 0

        self.data_lock.acquire()

        while self.serial_port.in_waiting > 0:
            rc = self.serial_port.read()
            # Check if first byte match Xsens Preamble byte (0xFA)
            if next_idx == 0:
                if rc == self.startbyte:
                    self.receivedBytes[0] = self.startbyte
                    next_idx = 1
                else:
                    next_idx = 0
                    if(self.ShowError):
                        print("Error read 1st byte, continue...")
            # Check if second byte match Xsens Bus Identifier (0xFF)
            elif next_idx == 1:
                if rc == self.secondbyte:
                    self.receivedBytes[1] = self.secondbyte
                    next_idx = 2
                else:
                    next_idx = 0
                    if(self.ShowError):
                        print("Error read 2nd byte, continue...")
            # Check if third byte is mtdata2
            elif next_idx == 2:
                if rc == self.mtdata2:
                    self.receivedBytes[2] = self.mtdata2
                    next_idx = 3
                else:
                    next_idx = 0
                    if(self.ShowError):
                        print("Error read 3rd byte, continue...")    
            elif next_idx == 3:
                # First 4 bytes + Last Checksum Byte = 5
                # 4th byte indicate the length of data field
                len_datafield = int.from_bytes(rc, 'big')
                self.len = len_datafield + 5
                self.receivedBytes[3] = rc
                next_idx = 4

                # Extend or Delete receivedBytes
                if(len(self.receivedBytes) < self.len):
                    extend_length = self.len - len(self.receivedBytes)
                    self.receivedBytes.extend([''] * extend_length)
                elif(len(self.receivedBytes) > self.len):
                    delete_length = len(self.receivedBytes) - self.len
                    del self.receivedBytes[-delete_length:]
            # If first three bytes checked, start reading
            else:
                if next_idx < self.len:
                    self.receivedBytes[next_idx] = rc 
                    next_idx += 1
                    # print(next_idx," ",int.from_bytes(rc , 'big'), rc.hex() ,self.receivedBytes[next_idx])
                
                if next_idx == self.len:
                    next_idx = 0
                    self.MarkDataNew()
                    break
        
        self.data_lock.release()

    def ParseData(self):
        CurrentIDX = self.MTData2StartIDX

        self.data_lock.acquire()
        while(CurrentIDX < len(self.receivedBytes)-1):
            # Check if valid header
            if self.receivedBytes[CurrentIDX] in self.TypeHex:
                MtdataType = self.receivedBytes[CurrentIDX]
                MtdataFormat = self.receivedBytes[CurrentIDX+1]
                MtdataLength = int.from_bytes(self.receivedBytes[CurrentIDX+2], 'big')
                MtdataMessageStart = CurrentIDX+3

                if MtdataType == b'\x08':
                    pass
                elif MtdataType == b'\x10':
                    pass
                elif MtdataType == b'\x20' and MtdataFormat == b'\x10' :
                    #-----LINK BYTES-----
                    Quat_Idx = MtdataMessageStart
                    self.quat_temp[0] = b''.join(self.receivedBytes[Quat_Idx+3  : Quat_Idx-1:-1])   # 3  2  1  0
                    self.quat_temp[1] = b''.join(self.receivedBytes[Quat_Idx+7  : Quat_Idx+3:-1])   # 7  6  5  4
                    self.quat_temp[2] = b''.join(self.receivedBytes[Quat_Idx+11 : Quat_Idx+7:-1])   # 11 10 9  8
                    self.quat_temp[3] = b''.join(self.receivedBytes[Quat_Idx+15 : Quat_Idx+11:-1])  # 15 14 13 12
                    #-----unpack data to float-----
                    self.quat[0] = struct.unpack('<f', self.quat_temp[0])[0]
                    self.quat[1] = struct.unpack('<f', self.quat_temp[1])[0]
                    self.quat[2] = struct.unpack('<f', self.quat_temp[2])[0]
                    self.quat[3] = struct.unpack('<f', self.quat_temp[3])[0]
                elif MtdataType == b'\x30':
                    pass
                elif MtdataType == b'\x40':
                    pass
                elif MtdataType == b'\x50':
                    pass
                elif MtdataType == b'\x70':
                    pass
                elif MtdataType == b'\x80':
                    pass
                elif MtdataType == b'\xA0':
                    pass
                elif MtdataType == b'\xC0':
                    pass
                elif MtdataType == b'\xD0':
                    pass
                elif MtdataType == b'\xE0':
                    pass
                else:
                    pass

                CurrentIDX = CurrentIDX + 2 + MtdataLength + 1  # Get Next Message Start Index
            else:
                pass
        self.data_lock.release()

    def QuatToEuler (self):  
        self.ParseData()
        w = self.quat[0]
        x = self.quat[1]
        y = self.quat[2]
        z = self.quat[3]
        
        # -----進行四方位角到euler 角的運算------
        # roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        self.euler[0,0] = math.atan2(sinr_cosp, cosr_cosp)
        # pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            self.euler[0,1] = math.copysign(math.pi / 2, sinp);  # use 90 degrees if out of range
        else:
            self.euler[0,1] = math.asin(sinp)
        #yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        self.euler[0,2] = math.atan2(siny_cosp, cosy_cosp)
