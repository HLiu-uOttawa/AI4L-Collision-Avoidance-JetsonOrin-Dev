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
                self.baud_rate = config.get("baud_rate", 115200)
                self.timeout = config.get("timeout", 1)
        except (FileNotFoundError, yaml.YAMLError) as e:
            print(f"Error loading config file: {e}")
            # Set default values if config fails
            self.port = None
            self.baud_rate = 115200
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

    def read_data(self, gps_current_data):
        """Read and parse multiple GPS messages per second (e.g. at 5Hz or 10Hz)."""
        if not self.serial_connection:
            print("No serial connection established.")
            return

        buffer = b""

        try:
            while True:
                # Read available bytes
                bytes_waiting = self.serial_connection.in_waiting
                if bytes_waiting > 0:
                    buffer += self.serial_connection.read(bytes_waiting)

                    # Split buffer into lines
                    lines = buffer.split(b'\n')
                    buffer = lines[-1]  # Keep last (incomplete) line
                    lines = lines[:-1]

                    for raw_line in lines:
                        try:
                            line = raw_line.decode('ascii', errors='ignore').strip()
                            if line.startswith("$GPGGA"):
                                print(f"[GPGGA] {line}")
                                msg = pynmea2.parse(line)
                                now = datetime.datetime.now()
                                timestamp = now.strftime("%Y-%m-%d_%H-%M-%S.%f")[:-3]
                                gps_data_line = f"{timestamp},{msg.latitude},{msg.longitude},{msg.altitude or 0.0},{msg.timestamp}"

                                script_dir = os.path.dirname(os.path.abspath(__file__))
                                file_path = os.path.join(script_dir, "gps_data", "gps_data.tmp")

                                os.makedirs(os.path.dirname(file_path), exist_ok=True)

                                if not os.path.exists(file_path):
                                    with open(file_path, 'w') as file:
                                        file.write("Timestamp, latitude, longitude, altitude, GpsTime\n")

                                gps_current_data.cur = gps_data_line

                                with open(file_path, 'a') as file:
                                    file.write(gps_data_line + '\n')

                        except pynmea2.ParseError:
                            continue

                time.sleep(0.01)  # Slight delay to reduce CPU load

        except KeyboardInterrupt:
            print("Exiting...")
        finally:
            self.disconnect()

    def disconnect(self):
        """Close the serial connection."""
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            print("Disconnected from GPS.")

    def set_gpgga_output_rate(self, rate_hz):
        """
        Set the output frequency of GPGGA message (NMEA) to match the update rate.
        Works on u-blox modules via UBX-CFG-MSG.
        """
        if not self.serial_connection or not self.serial_connection.is_open:
            print("Serial port is not open.")
            return

        msg_class = 0xF0  # NMEA message class
        msg_id = 0x00  # GPGGA
        rate = rate_hz  # Output rate on UART1

        # UBX-CFG-MSG structure: Class, ID, Rates[6]
        payload = bytes([msg_class, msg_id]) + bytes([rate, 0, 0, 0, 0, 0])

        # Header
        header = b'\xB5\x62\x06\x01' + len(payload).to_bytes(2, 'little')

        # Checksum
        ck_a = 0
        ck_b = 0
        for b in payload:
            ck_a = (ck_a + b) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF

        checksum = bytes([ck_a, ck_b])
        message = header + payload + checksum

        self.serial_connection.write(message)
        print(f"GPGGA message output rate set to {rate_hz} Hz.")

    def set_update_rate(self, rate_hz):
        """
        Set the GPS update rate (position output frequency) in Hz.
        This function sends a UBX-CFG-RATE command to u-blox-based GPS modules.

        :param rate_hz: Desired update rate in Hz, e.g., 1, 5, or 10.
        """
        if not self.serial_connection or not self.serial_connection.is_open:
            print("Serial port is not open. Cannot set update rate.")
            return

        # Calculate measurement rate in milliseconds (e.g., 5Hz = 200ms)
        meas_rate_ms = int(1000 / rate_hz)
        nav_rate = 1  # Number of measurements per navigation solution
        time_ref = 1  # 0 = UTC, 1 = GPS time

        # Build payload: measRate (2 bytes), navRate (2 bytes), timeRef (2 bytes)
        payload = meas_rate_ms.to_bytes(2, 'little') + \
                  nav_rate.to_bytes(2, 'little') + \
                  time_ref.to_bytes(2, 'little')

        # Build UBX-CFG-RATE message header
        header = b'\xB5\x62\x06\x08' + len(payload).to_bytes(2, 'little')

        # Compute checksum over the payload
        ck_a = 0
        ck_b = 0
        for b in payload:
            ck_a = (ck_a + b) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF

        checksum = bytes([ck_a, ck_b])
        message = header + payload + checksum

        # Send the command over serial
        self.serial_connection.write(message)
        print(f"GPS update rate set to {rate_hz} Hz.")


def gps_process(stop_event, gps_data_queue, gps_current_data):
    print("Starting gps process...")
    file_path = "./gps_data/gps_data.dat"
    if os.path.exists(file_path):
        creation_time = os.path.getctime(file_path)
        formatted_time = time.strftime('%Y-%m-%d-%H-%M-%S', time.localtime(creation_time))
        os.rename(file_path, "./gps_data/" + "gps_" + formatted_time + ".log")

    # # Provide the path to the configuration file
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_file_path = config_file_path = os.path.join(script_dir, "gps_config.yml")

    gps = GPS(config_file_path)

    gps.connect()
    # gps.set_update_rate(5)
    # gps.set_gpgga_output_rate(5)

    # gps.enable_gpgga()  # 发送 GPGGA 使能命令
    # gps.disable_gprmc()  # 关闭 GPRMC

    gps.read_data(gps_current_data)
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
