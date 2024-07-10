import pymavlink
from pymavlink import mavutil

class H7FC_Obj:
    imu_accessing = False
    imu_time = 0
    gyr_x = 0.0
    gyr_y = 0.0
    gyr_z = 0.0
    acc_x = 0.0
    acc_y = 0.0
    acc_z = 0.0
    attitude_accessing = False
    attitude_time = 0
    pitch = 0.0
    roll  = 0.0
    yaw   = 0.0
    baro_accessing = False
    baro_time  = 0
    baro_press = 0.0
    baro_alt   = 0.0

    def __init__(self, port_name):
        print(pymavlink.__doc__)
        self.mav_port = mavutil.mavlink_connection(port_name)

    def parse(self):
        msg = self.mav_port.recv_match(blocking = False)
        print(msg._type)

