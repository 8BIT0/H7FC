#!/usr/bin/env python3
import serial
import time
import queue
import pid_para
from enum import Enum

class CLI_State(Enum):
    CLI_No_Error    = 1
    CLI_Sending     = 2
    CLI_Error       = 3
    CLI_Parsing     = 4
    CLI_TimeOut     = 5

class CLI_Ctl:
    def __init__(self, port_obj):
        self.port = port_obj
        self.__rec_q = queue.Queue(8192)
        self.GyrXPID_Para = pid_para.PID_Param()
        self.GyrYPID_Para = pid_para.PID_Param()
        self.GyrZPID_Para = pid_para.PID_Param()

    def Into_CLI_Mode(self):
        if not self.port.is_open:
            print("[ COM port is not open ]")
            return CLI_State.CLI_Error
        
        for i in range(5):
            print("[ Send CMD to switch drone`s protocol ]")
            self.port.write(b"\r\n")
            
            # after send \r\n wait for 1sec
            time.sleep(1)
            
            sys_time = int(round(time.time()) * 1000)
            # check data reply from drone
            while True:
                buf = None
                if self.port.in_waiting:
                    buf = self.port.readline()
                
                if len(buf):
                    if buf.decode("ASCII").find("P.0.Wder Squad:/$") != -1:
                        print("[ Current protocol mode on done is CLI ]")
                        return CLI_State.CLI_No_Error
                    
                # check for time out
                if int(round(time.time()) * 1000) - sys_time >= 1000:
                    print("[ Drone protocol mode switch TIME OUT ]")
                    break;
        
        return CLI_State.CLI_TimeOut

    def __Controller_Param(self):
        # get controller type first
        # currently Attitude controller is CasecadePID
        # get inuse angular speed controller parameter
        if not self.port.is_open:
            print("[COM port is not open]")
            return CLI_State.CLI_Error

        self.port.write(b'show_inuse_pid\r\n')
        # parse drone reply
        while True:
            if self.port.in_waiting:
                pass

    def Get_Blackbox_Data(self):
        if not self.port.is_open:
            print("[COM port is not open]")
            return CLI_State.CLI_Error

        self.__Controller_Param(self)

        # create a file
        try:
            print("[Creating Log file]")
            log_file = open("log.txt", 'w')
        except:
            print("[Log file create filed]")
            return

        time.sleep(0.5)
        self.port.write(b"blackbox_info\r\n")
        pass

    def Set_BlackBox_LogType(self):
        pass
