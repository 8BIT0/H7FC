import serial
import time
import queue
from enum import Enum

class CLI_State(Enum):
    CLI_No_Error    = 1
    CLI_Sending     = 2
    CLI_Error       = 3
    CLI_Parsing     = 4

class CLI_Ctl:
    def __init__(self, port_obj):
        self.port = port_obj
        self.__rec_q = queue.Queue(8192)

    def Into_CLI_Mode(self):
        if not self.port.is_open:
            print("[COM port is not open]")
            return CLI_State.CLI_Error
            
        print("[Send CMD switch drone protocal into CLI mode]")
        self.port.write(b"\r\n")

        # after send \r\n wait for 1sec
        time.sleep(1)

        # check data reply from drone
        while True:
            buf = None
            if self.port.in_waiting:
                buf = self.port.readline()

            if len(buf):
                # chack data in queue
                print("[Receive data from drone]")
                print(buf.decode("ASCII"))

                # buf.decode("ASCII").find()

        return CLI_State.CLI_No_Error

    def __Controller_Param(self):
        # get controller type first
        # get angular speed controller parameter
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
