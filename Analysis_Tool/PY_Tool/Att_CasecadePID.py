from enum import Enum
import time
from pid_para import PID_Param as single_pid
from pid_para import PID_Parse_State as single_state

class Decode_Progress(Enum):
    Decode_None         = 0
    Decode_PitchPart    = 1
    Decode_RollPart     = 2
    Decode_GyroXPart    = 3
    Decode_GyroYPart    = 4
    Decode_GyroZPart    = 5

class ParaItem_Index(Enum):
    Item_Pitch  = 0
    Item_Roll   = 1
    Item_GyroX  = 2
    Item_GyroY  = 3
    Item_GyroZ  = 4

class Att_CaseCadePID:
    def __init__(self, port):
        self.__port = port
        self.__parse_num = 0
        self.decode_progress = Decode_Progress.Decode_None
        self.PitchPID_Para = single_pid()
        self.RollPID_Para = single_pid()
        self.GyrXPID_Para = single_pid()
        self.GyrYPID_Para = single_pid()
        self.GyrZPID_Para = single_pid()

    def __sys_ms(self):
        return int(time.time() * 1000)

    def __ack_finish(self, bytes):
        if len(bytes) and bytes.decode("ASCII").find("P.0.Wder Squad:/$") != -1:
            return True
        return False

    def __parse(self, bytes):
        if len(bytes) == 0:
            return False
        
        for i in bytes:
            if i.decode("ASCII").find("Pitch") != -1:
                if self.decode_progress != Decode_Progress.Decode_None:
                    return False
                self.decode_progress = Decode_Progress.Decode_PitchPart
                continue

            if i.decode("ASCII").find("Roll") != -1:
                print("[ Parsing roll parameter ]")
                if self.decode_progress != Decode_Progress.Decode_None:
                    return False
                self.decode_progress = Decode_Progress.Decode_RollPart
                continue

            if i.decode("ASCII").find("GyroX") != -1:
                print("[ Parsing gyro X parameter ]")
                if self.decode_progress != Decode_Progress.Decode_None:
                    return False
                self.decode_progress = Decode_Progress.Decode_GyroXPart
                continue
            
            if i.decode("ASCII").find("GyroY") != -1:
                print("[ Parsing gyro Y parameter ]")
                if self.decode_progress != Decode_Progress.Decode_None:
                    return False
                self.decode_progress = Decode_Progress.Decode_GyroYPart
                continue

            if i.decode("ASCII").find("GyroZ") != -1:
                print("[ Parsing gyro Z parameter ]")
                if self.decode_progress != Decode_Progress.Decode_None:
                    return False
                self.decode_progress = Decode_Progress.Decode_GyroZPart
                continue
            
            parse_state = single_state.Parse_None
            if self.decode_progress == Decode_Progress.Decode_PitchPart:
                parse_state = self.PitchPID_Para.parse(i)
                if parse_state == single_state.Parse_Fin:
                    self.decode_progress = Decode_Progress.Decode_None
                elif parse_state == single_state.Parse_Error:
                    print("[ Pitch parameter decode error ]")
                    return False

            elif self.decode_progress == Decode_Progress.Decode_RollPart:
                parse_state = self.RollPID_Para.parse(i)
                if parse_state == single_state.Parse_Fin:
                    self.decode_progress = Decode_Progress.Decode_None
                elif parse_state == single_state.Parse_Error:
                    print("[ Roll parameter decode error ]")
                    return False

            elif self.decode_progress == Decode_Progress.Decode_GyroXPart:
                parse_state = self.GyrXPID_Para.parse(i)
                if parse_state == single_state.Parse_Fin:
                    self.decode_progress = Decode_Progress.Decode_None
                elif parse_state == single_state.Parse_Error:
                    print("[ Gyro X parameter decode error ]")
                    return False

            elif self.decode_progress == Decode_Progress.Decode_GyroYPart:
                parse_state = self.GyrYPID_Para.parse(i)
                if parse_state == single_state.Parse_Fin:
                    self.decode_progress = Decode_Progress.Decode_None
                elif parse_state == single_state.Parse_Error:
                    print("[ Gyro Y parameter decode error ]")
                    return False

            elif self.decode_progress == Decode_Progress.Decode_GyroZPart:
                parse_state = self.GyrZPID_Para.parse(i)
                if parse_state == single_state.Parse_Fin:
                    self.decode_progress = Decode_Progress.Decode_None
                    self.__parse_num += 1
                    return True
                elif parse_state == single_state.Parse_Error:
                    print("[ Gyro Z parameter decode error ]")
                    return False

    def format_str(self):
        if self.__parse_num == 0:
            return ""
        
        str = "[ ---- PITCH ---- ]\t" + self.PitchPID_Para.format_str()
        str += "[ ---- ROLL ----- ]\t" + self.RollPID_Para.format_str()
        str += "[ ---- GyroX ---- ]\t" + self.GyrXPID_Para.format_str()
        str += "[ ---- GyroY ---- ]\t" + self.GyrYPID_Para.format_str()
        str += "[ ---- GyroZ ---- ]\t" + self.GyrZPID_Para.format_str()
        return str
    
    def get_value(self):
        para_list = []
        para_list.append(self.PitchPID_Para.get_val())
        para_list.append(self.RollPID_Para.get_val())
        para_list.append(self.GyrXPID_Para.get_val())
        para_list.append(self.GyrYPID_Para.get_val())
        para_list.append(self.GyrZPID_Para.get_val())
        return para_list

    def parse_para(self):
        # get controller type first
        # currently Attitude controller only CasecadePID
        # get inuse angular speed controller parameter        
        para = []
        self.__port.write(b'show_stored_pid\r\n')
        time.sleep(1)
        
        # parse drone reply
        sys_time = self.__sys_ms()
        reply = False
        self.PitchPID_Para.clear()
        self.RollPID_Para.clear()
        self.GyrXPID_Para.clear()
        self.GyrYPID_Para.clear()
        self.GyrZPID_Para.clear()
        while True:
            if self.__port.in_waiting:
                buf = self.__port.readline()
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
                        if not self.__parse(para):
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
