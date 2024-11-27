#!/usr/bin/env python3
import serial
import time
import Att_CasecadePID
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
        self.Att_PID = Att_CasecadePID.Att_CaseCadePID()

    def __ack_finish(self, bytes):
        if len(bytes) and bytes.decode("ASCII").find("P.0.Wder Squad:/$") != -1:
            return True
        return False
    
    def __sys_ms(self):
        return int(round(time.time()) * 1000)        

    def Into_CLI_Mode(self):
        if not self.port.is_open:
            print("[ COM port is not open ]")
            return CLI_State.CLI_Error
        
        # 5 times retry
        for i in range(5):
            print("[ Send CMD to switch drone`s protocol ]")
            self.port.write(b"\r\n")
            
            # after send \r\n wait for 1sec
            time.sleep(1)
            
            sys_time = self.__sys_ms()
            # check data reply from drone
            while True:
                buf = None
                if self.port.in_waiting:
                    buf = self.port.readline()
                
                if self.__ack_finish(buf):
                    print("[ Current protocol mode on done is CLI ]")
                    return CLI_State.CLI_No_Error
                    
                # check for time out
                if self.__sys_ms() - sys_time >= 1000:
                    print("[ Drone protocol mode switch TIME OUT ]")
                    break
        
        return CLI_State.CLI_TimeOut

    def __Controller_Param(self):
        # get controller type first
        # currently Attitude controller only CasecadePID
        # get inuse angular speed controller parameter        
        para = []
        self.port.write(b'show_inuse_pid\r\n')
        time.sleep(1)
        
        # parse drone reply
        sys_time = self.__sys_ms()
        reply = False
        while True:
            if self.port.in_waiting:
                buf = self.port.readline()
                if len(buf) and not reply:
                    if buf.decode("ASCII").find("[ ---- inuse parameter ---- ]") != -1:
                        print("[ Receiving controller parameter ]")
                        reply = True
                        continue
                
                if reply:
                    sys_time = self.__sys_ms()
                    if not self.__ack_finish(buf):
                        para.append(buf)
                    else :
                        print("[ Parsing controller parameter ]")
                        if not self.Att_PID.parse(para):
                            return False
                    # return True
            
            # check for receive time out (1S TimeOut)
            if self.__sys_ms() - sys_time > 1000:
                print("[ Drone controller parameter reply time out ]")
                return False
            
            time.sleep(0.01)

    def Get_Blackbox_Data(self):
        if not self.port.is_open:
            print("[COM port is not open]")
            return CLI_State.CLI_Error

        self.__Controller_Param()

        # create a file
        # try:
        #     print("[Creating Log file]")
        #     log_file = open("log.txt", 'w')

        #     self.port.write(b"blackbox_info\r\n")
        #     time.sleep(0.5)
        
        # except:
        #     print("[Log file create filed]")
        #     return
 
    def Set_BlackBox_LogType(self):
        pass
