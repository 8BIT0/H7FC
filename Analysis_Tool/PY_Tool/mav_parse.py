from pymavlink import mavutil

class H7FC_Obj:
    self.imu_accessing = False
    self.imu_time = 0
    self.gyr_x = 0.0
    self.gyr_y = 0.0
    self.gyr_z = 0.0
    self.acc_x = 0.0
    self.acc_y = 0.0
    self.acc_z = 0.0

    self.attitude_accessing = False
    self.attitude_time = 0
    self.pitch = 0.0
    self.roll  = 0.0
    self.yaw   = 0.0

    self.baro_accessing = False
    self.baro_time  = 0
    self.baro_press = 0.0
    self.baro_alt   = 0.0

    def __init__(self, port_name):
        self.mav_port = mavutil.mavlink_connection(port_name)

    def parse(self, byte):
        mavutil.recv_match()

