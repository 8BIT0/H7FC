#!/usr/bin/env python3
import time
import Att_CasecadePID
from enum import Enum
import tkinter as TK

class CLI_State(Enum):
    CLI_No_Error        = 1
    CLI_Sending         = 2
    CLI_Error           = 3
    CLI_Parsing         = 4
    CLI_TimeOut         = 5
    CLI_Parsing_Error   = 6

class CLI_Ctl:
    def __init__(self, port_obj):
        self.port = port_obj
        self.Att_PID = Att_CasecadePID.Att_CaseCadePID()
        self.tune_widget = False

    def __ack_finish(self, bytes):
        if len(bytes) and bytes.decode("ASCII").find("P.0.Wder Squad:/$") != -1:
            return True
        return False
    
    def __sys_ms(self):
        return int(time.time() * 1000)        

    def Quit_CLI(self):
        if not self.port.is_open:
            print("[ COM port is not open ]")
            return CLI_State.CLI_Errors
        self.port.write(b"CLI_Disable\r\n")
        return CLI_State.CLI_No_Error

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
                            print("[ Controller parameter parsing error ]")
                            return False
                        else:
                            print("[ Controller parameter parse successed ]")
                            return True
            
            # check for receive time out (1S TimeOut)
            if self.__sys_ms() - sys_time > 1000:
                print("[ Drone controller parameter reply time out ]")
                return False
            
            time.sleep(0.01)

    def Get_Blackbox_Data(self):
        if not self.port.is_open:
            print("[COM port is not open]")
            return CLI_State.CLI_Error

        if not self.__Controller_Param():
            return CLI_State.CLI_Parsing_Error

        # get date
        date = time.strftime("%y-%m-%d", time.localtime())
        
        # get log data type
        self.port.write(b"blackbox_type\r\n")
        time.sleep(0.5)
        sys_time = self.__sys_ms()
        while True:
            # check for time out
            if self.__sys_ms() - sys_time >= 1000:
                return
            
            if self.port.in_waiting:
                log_type = self.port.readline()
                if log_type.decode("ASCII").find("[ BlackBox ] log type: ") != -1:
                    type_str = log_type.decode("ASCII").rstrip("\r\n").split("[ BlackBox ] log type: ")[1]
                    print("[ BlackBox Log Type", type_str, "]")
                    break

        # create a file
        print("[ Creating Log file ]")
        log_file = open("log_" + type_str + "_" + date + ".txt", 'w')
        print("[ Log file created ]")

        # write parameter into file
        if not log_file.writable():
            print("[ log file not avaliable ]")
            return
            
        if type_str.find("AttitudePID") != -1 or type_str.find("AngularPID") != -1:
            format_para = self.Att_PID.format_str()
            print(format_para)
            log_file.write(format_para)
            log_file.write("[ ---- Type ----- ]\t" + type_str + "\r\n")
            log_file.write("-----------------------------------------------------------------------------\r\n")

        self.port.write(b"blackbox_info\r\n")
        time.sleep(0.5)

        sys_time = self.__sys_ms()
        log_start = False
        lines = []
        while True:
            # check for time out
            if self.__sys_ms() - sys_time >= 1000:
                print("[ BlackBox data read time out ]")
                break

            if self.port.in_waiting:
                sys_time = self.__sys_ms()
                log_str = self.port.readline()
                if log_str.decode("ASCII").find("[ BlackBox ] service log size:") != -1:
                    log_start = True
                    continue

                # receiving black box log data
                # if matched "[ BlackBox ] Log End" then finished
                if log_str.decode("ASCII").find("[ BlackBox ] Log End") != -1:
                    print("[ BlackBox log end ]")
                    log_file.writelines(lines)
                    log_start = False
                    break

                if log_start:
                    print(log_str)
                    if log_str.decode("ASCII").find("[ BlackBox ] ender error") != -1:
                        lines.pop()
                    else:
                        lines.append(log_str.decode("ASCII"))

        print("[ Close log file ]")
        log_file.close()

    def __tune_UI(self):
        if not self.tune_widget:
            # create UI widget
            pass
        pass

    def Tune_Controller(self):
        pass