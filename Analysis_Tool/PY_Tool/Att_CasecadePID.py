from enum import Enum
from pid_para import PID_Param as single_pid
from pid_para import PID_Parse_State as single_state

class Decode_Progress(Enum):
    Decode_None         = 0
    Decode_PitchPart    = 1
    Decode_RollPart     = 2
    Decode_GyroXPart    = 3
    Decode_GyroYPart    = 4
    Decode_GyroZPart    = 5

class Att_CaseCadePID:
    def __init__(self):
        self.full_str = []
        self.decode_progress = Decode_Progress.Decode_None
        self.PitchPID_Para = single_pid()
        self.RollPID_Para = single_pid()
        self.GyrXPID_Para = single_pid()
        self.GyrYPID_Para = single_pid()
        self.GyrZPID_Para = single_pid()

    def parse(self, bytes):
        if len(bytes) == 0:
            return False
        
        self.full_str = bytes
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
                    return True
                elif parse_state == single_state.Parse_Error:
                    print("[ Gyro Z parameter decode error ]")
                    return False

    def get(self):
        pass