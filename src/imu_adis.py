#!/usr/bin/env python

import sys
import serial
import serial.threaded
from serial.tools import list_ports
import time
from cobs import cobs

import threading
import thread
from time import sleep

from struct import *

from ctypes import *
import struct
from struct import *
import math

import crc_stm32

import rospy
from sensor_msgs.msg import Imu

########################################################################################################################
class MyForm:    
    class AHRSStructType(Structure):
        _fields_ = [("q0", c_float),
                    ("q1", c_float),
                    ("q2", c_float),
                    ("q3", c_float),
                    ("Roll", c_float),
                    ("Pitch", c_float),
                    ("Yaw", c_float),
                    ("Altitude", c_float),
                    ("Velocity", c_float),
                    ("Accel_x", c_float),
                    ("Accel_y", c_float),
                    ("Accel_z", c_float),
                    ("Gyro_x", c_float),
                    ("Gyro_y", c_float),
                    ("Gyro_z", c_float),
                    ("Magn_x", c_float),
                    ("Magn_y", c_float),
                    ("Magn_z", c_float),
                    ("Baro", c_float),
                    ("Temp", c_float),
                    ("dt", c_float),
                    ("dt_BMP", c_float)]

##################################################################################
#########################         __init__         ###############################
##################################################################################

    def __init__(self, parent=None):
        rospy.init_node('imu_adis')
        self.imuPub = rospy.Publisher('imu/adis', Imu, queue_size=1000)
        self.rate = rospy.Rate(100)
        self.msg = Imu()
        self.seq = 0;
        
        # Init crc calculation
        self.poly = 0x04C11DB7
        crc_stm32.generate_crc32_table(self.poly)

        # If there are COM ports detected
        if list_ports.comports():
            self.ser = serial.Serial
            self.connected = 0

  
        # Close serial if it is already open
        self.connected = 0

        # Struktura danych AHRS
        self.AHRSDataStruct = self.AHRSStructType

        # Receiveing
        self.no_of_bytes = 10
        self.kill_receive = threading.Event()
        self.kill_thread = threading.Event()
        self.thread = threading.Thread(target=self.Receive_Serial)
        self.kill_thread.set()
        self.kill_receive.set()

        self.rxBuffer = ""
        self.listcnt=0
        self.old_nb=0
        self.data=0
        self.dataIMU=0
        self.dataAHRS=0
        self.dataMB=0
        
              # Connect button to clicked actionself.AHRSDataStruct.Gyro_x
        self.Connect_Serial()
        self.SendSerial()

        # Start tasks
        # thread.start_new_thread(self.logData, ("Thread-2", 0.05,)) #zmienic na logowanie

##################################################################################
######################         Receive_Serial         ############################
##################################################################################

    def Receive_Serial(self):
        while not rospy.is_shutdown():
            while self.kill_thread.isSet():
                while self.kill_thread.isSet():
                    if self.connected:
                        try:
                            bytesToRead = self.ser.inWaiting()
                            if bytesToRead:
                                serin = self.ser.read(bytesToRead)
                                self.rxBuffer+=serin

                                #self.packets = self.rxBuffer.split('\x00')

                                if self.rxBuffer.find('\x00') >= 1:
                                    self.packets = self.rxBuffer.split('\x00')
                                    self.rxBuffer = self.rxBuffer[self.rxBuffer.find('\x00')+1:]
                                    
                                    decoded = cobs.decode(self.packets[0])

                                    package_designator = unpack('I', bytearray(decoded[:4]))

                                    ########################################################### IMU Packet ###########################################################
                                    if package_designator[0] == 0x7EEEEEEE:
                                        CRC = crc_stm32.custom_crc32(list(unpack('IIIIIIIIIIII', bytearray(decoded[:len(decoded) - 4]))))
                                        package_CRC = unpack('I', bytearray(decoded[len(decoded)-4:]))

                                        if CRC == package_CRC[0]:
                                            self.stucture_unpacked = unpack('IIffffffffffI', bytearray(decoded))
                                            
                                            if self.stucture_unpacked[1] - self.old_nb > 1:
                                                print 'Missing one packet'

                                            self.old_nb = self.stucture_unpacked[1]
                                            #print self.stucture_unpacked

                                            self.AHRSDataStruct.Gyro_x = self.stucture_unpacked[2]
                                            self.AHRSDataStruct.Gyro_y = self.stucture_unpacked[3]
                                            self.AHRSDataStruct.Gyro_z = self.stucture_unpacked[4]
                                            self.AHRSDataStruct.Accel_x = self.stucture_unpacked[5]
                                            self.AHRSDataStruct.Accel_y = self.stucture_unpacked[6]
                                            self.AHRSDataStruct.Accel_z = self.stucture_unpacked[7]
                                            self.AHRSDataStruct.Magn_x = self.stucture_unpacked[8]
                                            self.AHRSDataStruct.Magn_y = self.stucture_unpacked[9]
                                            self.AHRSDataStruct.Magn_z = self.stucture_unpacked[10]
                                            self.AHRSDataStruct.dt = self.stucture_unpacked[11]

                                            self.AHRSDataStruct.Baro = 0
                                            self.AHRSDataStruct.Temp = 0
                                            
                                            self.dataIMU = 1
                                            
                                            # Generating ROS Imu message
                                            
                                            self.msg.angular_velocity.x = self.AHRSDataStruct.Accel_x
                                            self.msg.angular_velocity.y = self.AHRSDataStruct.Accel_y
                                            self.msg.angular_velocity.z = self.AHRSDataStruct.Accel_z
                                            
                                            self.msg.linear_acceleration.x = self.AHRSDataStruct.Magn_x
                                            self.msg.linear_acceleration.y = self.AHRSDataStruct.Magn_y
                                            self.msg.linear_acceleration.z = self.AHRSDataStruct.Magn_z
                                            
                                        else:
                                            print "IMU CRC error"

                                    ########################################################### AHRS Packet ###########################################################
                                    if package_designator[0] == 0x6EEEEEEE:
                                        CRC = crc_stm32.custom_crc32(list(unpack('IIIIIIIII', bytearray(decoded[:len(decoded) - 4]))))
                                        package_CRC = unpack('I', bytearray(decoded[len(decoded)-4:]))

                                        if CRC == package_CRC[0]:
                                            self.stucture_unpacked = unpack('IIfffffffI', bytearray(decoded))

                                            self.AHRSDataStruct.q0 = self.stucture_unpacked[2]
                                            self.AHRSDataStruct.q1 = self.stucture_unpacked[3]
                                            self.AHRSDataStruct.q2 = self.stucture_unpacked[4]
                                            self.AHRSDataStruct.q3 = self.stucture_unpacked[5]
                                            self.AHRSDataStruct.Altitude = self.stucture_unpacked[6]
                                            self.AHRSDataStruct.Velocity = self.stucture_unpacked[7]
                                            self.AHRSDataStruct.dt = self.stucture_unpacked[8]

                                            self.dataAHRS = 1
                                            
                                            # Filling orientation information in the message
                                            
                                            self.msg.header.frame_id = 'imu'
                                            self.msg.header.stamp = rospy.Time.now()
                                            
                                            self.msg.orientation.x = self.AHRSDataStruct.q0
                                            self.msg.orientation.y = self.AHRSDataStruct.q1
                                            self.msg.orientation.z = self.AHRSDataStruct.q2
                                            self.msg.orientation.w = self.AHRSDataStruct.q3
                                            
                                            rospy.loginfo('Sending message')
                                            rospy.loginfo(self.msg)
                                            
                                            self.imuPub.publish(self.msg)
                                            self.rate.sleep()
                                        else:
                                            print "AHRS CRC error"

                        except Exception as e:
                            print e
                            print 'COM port disconnected'
                            pass

                        sleep(0.0001)


##################################################################################
########################         SendSerial         ##############################
##################################################################################

    def SendSerial(self):
        if self.connected:
            self.ser.write("abcdefg!")

##################################################################################
######################         Connect_Serial         ############################
##################################################################################

    def Connect_Serial(self):
        if not self.connected:
            # Connect to selected COM port
            self.ser = serial.Serial(port='/dev/ttyACM0', baudrate=460800, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
            # Close COM port if it is already open
            if self.ser.isOpen():
                self.ser.close()
                self.connected = 0

            # Open COM port
            self.ser.open()
            # If open successful
            if self.ser.isOpen():
                self.connected = 1
                print ('Connected to: ' + self.ser.portstr)
                try:
                    self.thread.start()
                except:
                    print 'connect serial'
                    pass
                self.kill_receive.set()

        else:
            self.kill_receive.clear()
            self.ser.close()
            self.connected = 0
            self.rxBuffer=self.rxBuffer[len(self.rxBuffer):]
            print ('Disconnected')

##################################################################################
####################         RefreshGUI_handler        ###########################
##################################################################################

    def logData(self, threadName, delay):
        if threadName == "Thread-2":
            while 1:
                time.sleep(delay)
                if self.connected:
                    if self.dataIMU:
                        print 'New dataIMU package'
                        print(self.AHRSDataStruct.Gyro_x)
                        print(self.AHRSDataStruct.Gyro_y)
                        print(self.AHRSDataStruct.Gyro_z)
                        print(self.AHRSDataStruct.Accel_x)
                        print(self.AHRSDataStruct.Accel_y)
                        print(self.AHRSDataStruct.Accel_z)
                        print(self.AHRSDataStruct.Magn_x)
                        print(self.AHRSDataStruct.Magn_y)
                        print(self.AHRSDataStruct.Magn_z)
                        print(self.AHRSDataStruct.Baro)
                        print(self.AHRSDataStruct.Temp)
                        print(self.AHRSDataStruct.dt)
                    if self.dataAHRS:
                        print 'New dataAHRS package'
                        print(self.AHRSDataStruct.q0)
                        print(self.AHRSDataStruct.q1)
                        print(self.AHRSDataStruct.q2)
                        print(self.AHRSDataStruct.q3)
                        print(self.AHRSDataStruct.Altitude)
                        print(self.AHRSDataStruct.Velocity)



##################################################################################
##########################         killAll         ###############################
##################################################################################

    def killAll(self):
        self.kill_receive.clear()
        self.kill_thread.clear()
        if self.ser.isOpen():
            self.ser.close()
            print ('Disconnected')

########################################################################################################################


myapp = MyForm()







