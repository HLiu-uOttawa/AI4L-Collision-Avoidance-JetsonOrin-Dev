import time, datetime, platform, os
import serial
import serial.tools.list_ports
import psutil

# import lib.device_model as deviceModel
from imu.lib.device_model import DeviceModel

from imu.lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
from imu.lib.protocol_resolver.roles.wit_protocol_resolver import WitProtocolResolver

from utils.log_utils import log_print

def setConfig(device):
    """
    Example setting configuration information
    :param device: Device model
    :return:
    """
    device.writeReg(0x52)  # Set the Z-axis angle to zero.
    time.sleep(0.1)  # Sleep 100ms

    device.writeReg(0x66)  # Set installation direction: Vertical.
    time.sleep(0.1)  # Sleep 100ms

    device.writeReg(0x65)  # Set installation direction: Horizontal.
    time.sleep(0.1)  # Sleep 100ms

    # Set the baud rate to 115200. After setting, wait for three seconds.
    # During this time, no data will be returned.
    # You need to switch to the corresponding baud rate below and rerun the program.
    device.writeReg(0x63)
    time.sleep(0.1)  # Sleep 100ms

    # Set the baud rate to 9600. After setting, wait for three seconds.
    # During this time, no data will be returned.
    # You need to switch to the corresponding baud rate below and rerun the program.
    device.writeReg(0x64)
    time.sleep(0.1)  # Sleep 100ms


def AccelerationCalibration(device):
    """
    Acceleration calibration
    :param device: Device model
    :return:
    """
    device.AccelerationCalibration()  # Acceleration calibration
    print("Acceleration calibration completed")


def onUpdate(dev, imu_status):
    """
    Data update event
    :param dev: Device model
    :return:
    """

    imu_data_content_temp = str(dev.getDeviceData("temperature"))
    imu_data_content_accl = str(dev.getDeviceData("accX")) + ";" + \
                            str(dev.getDeviceData("accY")) + ";" + \
                            str(dev.getDeviceData("accZ"))

    # Angular Velocity
    imu_data_content_gyro = str(dev.getDeviceData("gyroX")) + ";" + \
                            str(dev.getDeviceData("gyroY")) + ";" + \
                            str(dev.getDeviceData("gyroZ"))

    imu_data_content_angle = str(dev.getDeviceData("angleX")) + ";" + \
                             str(dev.getDeviceData("angleY")) + ";" + \
                             str(dev.getDeviceData("angleZ"))

    now = datetime.datetime.now()
    timestamp = now.strftime("%Y-%m-%d_%H-%M-%S.%f")[:-3]
    imu_data_line = timestamp + "," + \
                    imu_data_content_temp + "," + \
                    imu_data_content_accl + "," + \
                    imu_data_content_gyro + "," + \
                    imu_data_content_angle



    script_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = "./imu_data/imu_data.tmp"
    file_path = os.path.join(script_dir, file_path)



    imu_status.state = "Here is test for imu status string!"
    imu_status.state = imu_data_line

    if not os.path.exists(file_path):
        imu_data_head = "Time, Temperature, Acceleration, Angular_Velocity, Angle"
        with open(file_path, 'w') as file:
            file.write(imu_data_head + '\n')
            # print(imu_data_head)
    else:
        # if file exists, append one line of data
        with open(file_path, 'a') as file:
            file.write(imu_data_line + '\n')
            # print(imu_data_line)

import time
from functools import partial

def main(imu_status):
    # ports = serial.tools.list_ports.comports()
    # for port in ports:
    #     print("List COM ports: ", port.device)

    """
    Initialize a device model
    """
    device = DeviceModel(
        "IMU_BWT61CL",
        WitProtocolResolver(),
        JY901SDataProcessor(),
        "51_0"
    )

    if platform.system().lower() == 'linux':
        device.serialConfig.portName = "/dev/ttyUSB0"  # Set serial port
    else:
        device.serialConfig.portName = "COM3"  # Set serial port
    device.serialConfig.baud = 115200  # Set baud rate
    device.openDevice()  # Open serial port
    # device.AccelerationCalibration()  # Acceleration calibration
    # print("Calibration end")
    # device.writeReg(0x52)  # set z-axis degree to be zone
    # device.writeReg(0x65)  # Set installation orientation: horizontal
    # device.writeReg(0x66)  # Set installation orientation: vertical
    # device.writeReg(0x63)  # Set baud rate: 115200
    # device.writeReg(0x64)  # Set baud rate: 9600
    device.dataProcessor.onVarChanged.append(
            partial(onUpdate, imu_status=imu_status)
            )  # Data update event

    # wait for stop signal
    # device.closeDevice()


'''
    startRecord()                                       # Start recording data
    input()
    device.closeDevice()
    endRecord()                                         # End record data
'''
def imu_process(stop_event, imu_data_queue, imu_status):
    # print("Starting imu process...")
    log_print("Starting imu process...", log_file_path="logs/common.log")


    file_path = "./imu_data/imu_data.tmp"

    if os.path.exists(file_path):
        creation_time = os.path.getctime(file_path)
        formatted_time = time.strftime('%Y-%m-%d-%H-%M-%S', time.localtime(creation_time))
        os.rename(file_path, "./imu_data/" + "imu_" + formatted_time + ".log")

    # Provide the path to the configuration file
    config_file_path = "imu_config.yml"

    main(imu_status)


if __name__ == '__main__':
    ...

