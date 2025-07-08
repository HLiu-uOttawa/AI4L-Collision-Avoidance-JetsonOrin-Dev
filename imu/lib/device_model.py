# coding:UTF-8
import threading
import _thread
import time
import struct
import serial
import serial.tools.list_ports
from serial import SerialException

from utils.log_utils import log_print

'''
    Serial port parameter configuration
'''


class SerialConfig:

    # port
    portName = ''

    # Baud rate
    baud = 9600

'''
Device model
'''


class DeviceModel:
    # Device name
    deviceName = "My Device"

    # Device ID
    ADDR = 0x50

    # Device data dictionary
    deviceData = {}

    isOpen = False

    # Serial port
    serialPort = None

    # Serial port configuration
    serialConfig = SerialConfig()

    # Update trigger
    dataUpdateListener = ""

    # Data parser
    dataProcessor = None

    # Protocol parser
    protocolResolver = None

    def __init__(self, deviceName, protocolResolver, dataProcessor, dataUpdateListener):
        # print("Initializing Device Model")
        log_print("Initializing Device Model", log_file_path="logs/common.log")

        self.deviceName = deviceName
        self.protocolResolver = protocolResolver
        self.dataProcessor = dataProcessor
        self.dataUpdateListener = dataUpdateListener
        # _thread.start_new_thread(self.readDataTh, ("Data-Received-Thread", 10, ))

    def setDeviceData(self, key, value):
        """
        Configure device data
        :param key: Data key
        :param value: Data value
        :return: No return value
        """
        self.deviceData[key] = value

    def getDeviceData(self, key):
        """
        Retrieve device data
        :param key: Data key
        :return: Return the data value; if the key does not exist, return None.
        """
        if ( key in self.deviceData):
            return self.deviceData[key]
        else:
            return None

    def removeDeviceData(self, key):
        """
        Delete device data
        :param key: Data key
        :return: No return value
        """
        del self.deviceData[key]

    def readDataTh(self, threadName, delay):
        """
        Thread for reading data
        :return:
        """
        print("Start " + threadName)
        while True:
            # If the serial port is open
            if self.isOpen:
                try:
                    tlen = self.serialPort.inWaiting()
                    if (tlen>0):
                        data = self.serialPort.read(tlen)
                        self.onDataReceived(data)
                except Exception as ex:
                    print(ex)
            else:
                time.sleep(0.1)
                print("Pausing")
                break

    def openDevice(self):
        """
                Attempts to open the serial port connection for the device.

                This method first closes any existing serial connection to prevent conflicts.
                It then attempts to open the serial port specified in `self.serialConfig.portName`
                using the configured baud rate (`self.serialConfig.baud`). If the port is successfully
                opened, a background thread is started to continuously read incoming data.

                Key Features:
                - Automatically closes any previously opened port
                - Checks whether the specified port exists on the system
                - Provides clear error messages if the port cannot be found or opened
                - Starts a separate thread for non-blocking serial data reading upon success

                Notes:
                - Ensure the device is properly connected before calling this method
                - Set the correct port name (e.g., "COM3" on Windows or "/dev/ttyUSB0" on Linux)
                - If the port is already in use by another program, opening will fail

                :return: None
                """

        self.closeDevice()

        # print(f"Try to connect IMU on {self.serialConfig.portName} ----------------")
        log_print(f"Try to connect IMU on {self.serialConfig.portName} ----------------", log_file_path="logs/common.log")
        
        available_ports = [p.device for p in serial.tools.list_ports.comports()]
        if self.serialConfig.portName not in available_ports:
            # print(f"[Error] Serial port '{self.serialConfig.portName}' not found.")
            log_print(f"[Error] Serial port '{self.serialConfig.portName}' not found.", log_file_path="logs/common.log")
            return
        try:
            self.serialPort = serial.Serial(
                self.serialConfig.portName,
                self.serialConfig.baud,
                timeout=0.5)
            self.isOpen = True
            # Start a thread to receive data.
            t = threading.Thread(target=self.readDataTh, args=("Data-Received-Thread",10,))
            t.start()
            print(f"[Info] Device opened on {self.serialConfig.portName} at {self.serialConfig.baud} baud.")
        except SerialException as e:
            print(f"[Error] Failed to open {self.serialConfig.portName} ({self.serialConfig.baud}): {e}")
            print("Please check if the port is occupied or the device is connected properly.")
            # print("Open" + self.serialConfig.portName + self.serialConfig.baud + "Failed")

    def closeDevice(self):
        """
        Close device
        :return: no return
        """
        if self.serialPort is not None:
            self.serialPort.close()
            print("Port Closed")
        self.isOpen = False
        # print("Device Close")
        log_print("Device Closed", log_file_path="logs/common.log")

    def onDataReceived(self, data):
        """
        When receiving data
        :param data: Received data
        :return: No return value
        """
        if self.protocolResolver is not None:
            self.protocolResolver.passiveReceiveData(data, self)

    def get_int(self,dataBytes):
        """
        Convert to signed integer (int) = C# BitConverter.ToInt16
        :param dataBytes: Byte array
        :return:
        """
        #return -(data & 0x8000) | (data & 0x7fff)
        return  int.from_bytes(dataBytes, "little", signed=True)

    def get_unint(self,dataBytes):
        """
        Convert a signed integer to an unsigned integer
        :param data:
        :return:
        """
        return  int.from_bytes(dataBytes, "little")


    def sendData(self, data):
        """
        Send data
        :return: Whether the data was sent successfully
        """
        if self.protocolResolver is not None:
            self.protocolResolver.sendData(data, self)

    def readReg(self, regAddr,regCount):
        """
        Register Read
        :param regAddr: register address
        :param regCount: Number of registers
        :return:
        """
        if self.protocolResolver is not None:
            return self.protocolResolver.readReg(regAddr,regCount, self)
        else:
            return

    def writeReg(self, regAddr):
        """
        Write to register
        :param regAddr: Register address
        :param sValue: Writing value
        :return:
        """
        if self.protocolResolver is not None:
            self.protocolResolver.writeReg(regAddr, self)

    def unlock(self):
        """
        unlock
        :return:
        """
        if self.protocolResolver is not None:
            self.protocolResolver.unlock(self)

    def save(self):
        """
        save device data
        :return:
        """
        if self.protocolResolver is not None:
            self.protocolResolver.save(self)

    def AccelerationCalibration(self):
        """
        Accelerometer calibration
        :return:
        """
        if self.protocolResolver is not None:
            self.protocolResolver.AccelerationCalibration(self)

    def BeginFiledCalibration(self):
        """
        Start magnetometer calibration
        :return:
        """
        if self.protocolResolver is not None:
            self.protocolResolver.BeginFiledCalibration(self)

    def EndFiledCalibration(self):
        """
        End magnetometer calibration
        :return:
        """
        if self.protocolResolver is not None:
            self.protocolResolver.EndFiledCalibration(self)

    def sendProtocolData(self, data):
        """
        Send protocol-based data
        :return:
        """
        if self.protocolResolver is not None:
            self.protocolResolver.sendData(data)


