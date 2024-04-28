import socket
import threading
import time
from pymavlink import mavutil
import struct


def float_to_bytes(float_value):
    return struct.pack('<f', float_value).hex()


def hex_to_float(hex_str):
    int_value = int(hex_str, 16)
    float_value = struct.unpack('<f', struct.pack('<I', int_value))[0]
    return float_value


class MavlinkDataForwarder:
    def __init__(self, mavlink_connection_address, read_host, read_port, write_host, write_port):
        super().__init__()
        self.mavlink_connection_address = mavlink_connection_address
        self.mavlink_connection = None

        self.read_host = read_host
        self.read_port = read_port
        self.write_host = write_host
        self.write_port = write_port
        self.read_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.write_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.read_sock.bind((self.read_host, self.read_port))

        self.latitude = 0.0
        self.longitude = 0.0
        self.msl = 0.0
        self.agl = 0.0
        self.home_latitude = 0.0
        self.home_longitude = 0.0
        self.home_msl = 0.0
        self.velocity_H = 0.0
        self.velocity_V = 0.0
        self.heading = 0.0
        self.gps = 0
        self.mode = 0
        self.arm_status = 2
        self.battery_voltage = 0.0
        self.battery_percent = 0
        self.mode_input = 0
        self.lock = threading.Lock()

    def requestMessage(self, msg, hz):
        self.mavlink_connection.mav.command_long_send(self.mavlink_connection.target_system,
                                                      self.mavlink_connection.target_component,
                                                      mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                                      0, msg, hz, 0, 0, 0, 0, 0)

    def get_location(self, msg):
        latitude = self.latitude
        longitude = self.longitude
        msl = self.msl
        agl = self.agl
        vz = self.velocity_V
        heading = self.heading

        if msg.get_type() == 'GLOBAL_POSITION_INT':
            latitude = msg.lat / 1e7
            longitude = msg.lon / 1e7
            msl = msg.alt / 1000
            agl = msg.relative_alt / 1000
            vz = msg.vz / 100
            heading = msg.hdg / 100
        return latitude, longitude, msl, agl, vz, heading

    def get_battery_status(self, msg):
        battery_voltage = self.battery_voltage
        battery_percent = self.battery_percent
        if msg.get_type() == 'BATTERY_STATUS':
            battery_voltage = msg.voltages[0] / 1000.0
            battery_percent = msg.battery_remaining

        return battery_voltage, battery_percent

    def get_gps(self, msg):
        gps = self.gps
        v_h = self.velocity_H
        if msg.get_type() == 'GPS_RAW_INT':
            v_h = msg.to_dict()["vel"] / 100
            gps = msg.to_dict()["satellites_visible"]
        return gps, v_h

    def get_home_location(self, msg):
        home_latitude = self.home_latitude
        home_longitude = self.home_longitude
        home_altitude = self.home_msl
        if msg.get_type() == 'HOME_POSITION':
            home_latitude = msg.latitude / 1e7
            home_longitude = msg.longitude / 1e7
            home_altitude = msg.altitude / 1000
        return home_latitude, home_longitude, home_altitude

    def get_mode(self, msg):
        mode = self.mode
        arm_status = self.arm_status
        if msg.get_type() == 'HEARTBEAT':
            mode = msg.to_dict()['custom_mode']
            vehicle_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

            if vehicle_armed:
                arm_status = 1
            else:
                arm_status = 0
        return mode, arm_status

    def read_mavlink_data(self):
        while not self.mavlink_connection:
            try:
                self.mavlink_connection = mavutil.mavlink_connection(self.mavlink_connection_address)
                self.mavlink_connection.wait_heartbeat()
                print("Connection Okay.")
            except Exception as e:
                print("Connection failed. Retrying...")
                time.sleep(1)
        self.mavlink_connection.wait_heartbeat()
        self.requestMessage(0, 1)  # HEARTBEAT
        self.requestMessage(24, 1)  # GPS_RAW_INT
        self.requestMessage(30, 1)  # ATTITUDE
        self.requestMessage(33, 1)  # GLOBAL_POSITION_INT
        self.requestMessage(147, 1)  # BATTERY_STATUS
        self.requestMessage(253, 1)  # STATUS_TEXT
        self.requestMessage(242, 1)  # HOME_POSITION

        while True:
            msg = self.mavlink_connection.recv_match()
            if msg:
                with self.lock:
                    (self.latitude, self.longitude, self.msl, self.agl, self.velocity_V,
                     self.heading) = self.get_location(msg)
                    self.gps, self.velocity_H = self.get_gps(msg)
                    self.mode, self.arm_status = self.get_mode(msg)
                    self.home_latitude, self.home_longitude, self.home_msl = self.get_home_location(msg)
                    self.battery_voltage, self.battery_percent = self.get_battery_status(msg)

    def send_udp_data(self):
        while True:
            with self.lock:
                latitude_bytes = bytes.fromhex(float_to_bytes(self.latitude))
                longitude_bytes = bytes.fromhex(float_to_bytes(self.longitude))
                msl_bytes = bytes.fromhex(float_to_bytes(self.msl))
                agl_bytes = bytes.fromhex(float_to_bytes(self.agl))
                home_latitude_bytes = bytes.fromhex(float_to_bytes(self.home_latitude))
                home_longitude_bytes = bytes.fromhex(float_to_bytes(self.home_longitude))
                home_msl_bytes = bytes.fromhex(float_to_bytes(self.home_msl))
                velocity_h_bytes = bytes.fromhex(float_to_bytes(self.velocity_H))
                velocity_v_bytes = bytes.fromhex(float_to_bytes(self.velocity_V))
                heading_bytes = bytes.fromhex(float_to_bytes(self.heading))
                gps_bytes = self.gps.to_bytes(1, byteorder='big')
                mode_bytes = self.mode.to_bytes(1, byteorder='big')
                arm_status_bytes = self.arm_status.to_bytes(1, byteorder='big')
                battery_voltage_bytes = bytes.fromhex(float_to_bytes(self.battery_voltage))
                battery_percent_bytes = self.battery_percent.to_bytes(1, byteorder='big')

                udp_sending_data = (b"\x4f" + latitude_bytes + longitude_bytes + msl_bytes + agl_bytes +
                                    home_latitude_bytes + home_longitude_bytes + home_msl_bytes +
                                    velocity_h_bytes + velocity_v_bytes + heading_bytes + gps_bytes + mode_bytes +
                                    arm_status_bytes + battery_voltage_bytes + battery_percent_bytes)

            time.sleep(0.1)
            self.write_sock.sendto(udp_sending_data, (self.write_host, self.write_port))

    def read_udp_data(self):

        while True:
            time.sleep(0.1)
            data, addr = self.read_sock.recvfrom(16)
            for i in data:
                print(hex(i))
            print(f"Received message from {addr}: {data}")

    def start(self):
        read_mavlink_thread = threading.Thread(target=self.read_mavlink_data)
        send_udp_thread = threading.Thread(target=self.send_udp_data)
        read_udp_thread=threading.Thread(target=self.read_udp_data)

        read_mavlink_thread.daemon = True
        send_udp_thread.daemon = True
        read_udp_thread.daemon=True

        read_mavlink_thread.start()
        send_udp_thread.start()
        read_udp_thread.start()

        read_mavlink_thread.join()
        send_udp_thread.join()
        read_udp_thread.join()


if __name__ == "__main__":
    mavlink_connection_address = 'tcp:127.0.0.1:5763'
    # udp_destination_address = ('192.168.1.134', 5801)
    forwarder = MavlinkDataForwarder(mavlink_connection_address, '192.168.1.116', 2601, '192.168.1.134', 5801)
    forwarder.start()
