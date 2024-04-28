import threading
import time
from pymavlink import mavutil


class MavlinkSDK:
    def __init__(self, mavlink_connection_address):
        super().__init__()
        self.mavlink_connection_address = mavlink_connection_address
        self.mavlink_connection = None
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
        self.connection_flag=False

    def request_message(self, msg, hz):
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
                self.connection_flag=True
                print("Connection Okay.")
            except (ConnectionError, OSError) as e:
                self.connection_flag=False
                print(f"Connection failed: {e} Retrying...")
                time.sleep(1)
        self.mavlink_connection.wait_heartbeat()
        self.request_message(0, 1)  # HEARTBEAT
        self.request_message(24, 1)  # GPS_RAW_INT
        self.request_message(30, 1)  # ATTITUDE
        self.request_message(33, 1)  # GLOBAL_POSITION_INT
        self.request_message(147, 1)  # BATTERY_STATUS
        self.request_message(253, 1)  # STATUS_TEXT
        self.request_message(242, 1)  # HOME_POSITION

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

    def read(self):
        while True:
            with self.lock:
                if self.connection_flag:
                    print("Latitude:", self.latitude)
                    print("Longitude:", self.longitude)
            time.sleep(0.1)

    def start(self):
        read_mavlink_thread = threading.Thread(target=self.read_mavlink_data)
        read_thread = threading.Thread(target=self.read)
        read_mavlink_thread.daemon = True
        read_thread.daemon = True
        read_mavlink_thread.start()
        read_thread.start()
        read_mavlink_thread.join()
        read_thread.join()


if __name__ == "__main__":
    uav_obj = MavlinkSDK('tcp:127.0.0.1:5763')
    uav_obj.start()
