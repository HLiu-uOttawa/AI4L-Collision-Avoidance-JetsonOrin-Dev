import os, time, datetime
import pynmea2
import yaml
import serial.tools.list_ports
# import serial
import multiprocessing as mp


class GPS:
    def __init__(self, config_path):
        self.load_config(config_path)
        self.serial_connection = None

    def load_config(self, config_path):
        """Load configuration from a YAML file."""
        try:
            with open(config_path, 'r') as config_file:
                config = yaml.safe_load(config_file)
                self.port = config.get("port")  # Allow port to be None initially
                self.baud_rate = config.get("baud_rate", 9600)
                self.timeout = config.get("timeout", 1)
        except (FileNotFoundError, yaml.YAMLError) as e:
            print(f"Error loading config file: {e}")
            # Set default values if config fails
            self.port = None
            self.baud_rate = 9600
            self.timeout = 1

    def list_ports(self):
        """List available COM ports."""
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]

    def auto_select_port(self):
        """Automatically select the GPS port."""
        available_ports = self.list_ports()
        print("Available ports:", available_ports)

        for port in available_ports:
            try:
                with serial.Serial(port, self.baud_rate, timeout=self.timeout) as ser:
                    print(f"Testing port: {port}")
                    line = ser.readline().decode('ascii', errors='ignore').strip()
                    if line.startswith("$GPGGA") or line.startswith("$GPRMC"):
                        print(f"GPS detected on port: {port}")
                        self.port = port
                        return
            except (serial.SerialException, UnicodeDecodeError):
                continue

        print("No GPS device found.")

    def connect(self):
        """Establish a connection to the GPS device."""
        if not self.port:
            print("No port specified. Attempting to auto-select.")
            self.auto_select_port()

        if not self.port:
            print("No GPS device found. Cannot connect.")
            return

        try:
            self.serial_connection = serial.Serial(self.port, self.baud_rate, timeout=self.timeout)
            print(f"Connected to GPS on {self.port}")
        except serial.SerialException as e:
            print(f"Error connecting to GPS: {e}")
            self.serial_connection = None

    # def send_ubx_command(self, command):
    #     """发送 UBX 命令到 GPS 模块"""
    #     if self.serial_connection.is_open:
    #         self.serial_connection.write(command)
    #         time.sleep(0.1)  # 等待 GPS 模块处理命令
    #         print(f"已发送 UBX 命令: {command.hex().upper()}")
    #     else:
    #         print("错误: GPS 设备未连接")
    #
    # def enable_gpgga(self):
    #     """启用 GPGGA ($GPGGA) 消息"""
    #     UBX_GGA_ON = bytes.fromhex("B5 62 06 01 08 00 F0 00 00 01 00 00 00 00 FF 23")
    #     self.send_ubx_command(UBX_GGA_ON)
    #
    # def disable_gprmc(self):
    #     """禁用 GPRMC ($GPRMC) 消息"""
    #     UBX_RMC_OFF = bytes.fromhex("B5 62 06 01 08 00 F0 04 00 00 00 00 00 00 FF 2B")
    #     self.send_ubx_command(UBX_RMC_OFF)

    def read_data(self):
        """Read and parse GPS data."""
        if not self.serial_connection:
            print("No serial connection established.")
            return

        try:
            while True:
                line = self.serial_connection.readline().decode('ascii', errors='ignore').strip()
                # print(line)
                # lines = self.serial_connection.readlines()
                # print(lines)

                # if line.startswith("$GPGGA") or line.startswith("$GPRMC"):
                if line.startswith("$GPRMC"):
                    ...
                if line.startswith("$GPGGA"):
                    # print(line)
                    try:
                        msg = pynmea2.parse(line)
                        # print(f"Latitude: {msg.latitude}, Longitude: {msg.longitude}, Time: {msg.timestamp}")
                        now = datetime.datetime.now()
                        timestamp = now.strftime("%Y-%m-%d_%H-%M-%S.%f")[:-3]
                        gps_data_line = timestamp + "," + \
                                        str(msg.latitude) + "," + \
                                        str(msg.longitude) + "," + \
                                        str(msg.altitude) + "," + \
                                        str(msg.timestamp)

                        script_dir = os.path.dirname(os.path.abspath(__file__))
                        file_path = "./gps_data/gps_data.tmp"
                        file_path = os.path.join(script_dir, file_path)

                        if not os.path.exists(file_path):
                            gps_data_head = "Timestamp, latitude, longitude, altitude, GpsTime"
                            with open(file_path, 'w') as file:
                                file.write(gps_data_head + '\n')
                                # print(gps_data_head)
                        else:
                            # if file exists, append one line of data
                            with open(file_path, 'a') as file:
                                file.write(gps_data_line + '\n')
                                # print(gps_data_line)
                    except pynmea2.ParseError as e:
                        print(f"Parse error: {e}")
        except KeyboardInterrupt:
            print("Exiting...")
        finally:
            self.disconnect()

    def disconnect(self):
        """Close the serial connection."""
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            print("Disconnected from GPS.")


def gps_process(stop_event, gps_data_queue):
    print("Starting gps process...")
    file_path = "./gps_data/gps_data.tmp"
    if os.path.exists(file_path):
        creation_time = os.path.getctime(file_path)
        formatted_time = time.strftime('%Y-%m-%d-%H-%M-%S', time.localtime(creation_time))
        os.rename(file_path, "./gps_data/" + "gps_" + formatted_time + ".log")

    # # Provide the path to the configuration file
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_file_path = config_file_path = os.path.join(script_dir, "gps_config.yml")

    gps = GPS(config_file_path)

    gps.connect()

    # gps.enable_gpgga()  # 发送 GPGGA 使能命令
    # gps.disable_gprmc()  # 关闭 GPRMC

    gps.read_data()
    # ...


def main():
    ...


# Example usage
if __name__ == '__main__':
    stop_event = mp.Event()
    gps_data_queue = mp.Queue()

    if True:
        print("Process gps and save data...")
        gps_proc = mp.Process(name="gps_processing", target=gps_process, args=(stop_event, gps_data_queue,))
        gps_proc.start()
        gps_proc.join()

    # try:
    #     while True:
    #         user_input = input("Type 'q' and hit ENTER to quit:\n")
    #         if user_input.lower() == 'q':
    #             stop_event.set()
    #             break
    # except KeyboardInterrupt:
    #     stop_event.set()
    # finally:
    #     if True:
    #         gps_proc.join()
