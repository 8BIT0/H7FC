from enum import Enum
from pid_para import PID_Param as single_pid

class Decode_Progress(Enum):
    Decode_None         = 0
    Decode_PitchPart    = 1
    Decode_RollPart     = 2
    Decode_GyroXPart    = 3
    Decode_GyroYPart    = 4
    Decode_GyroZPart    = 5

class Att_CaseCadePID:
    def __init__(self):
        self.decode_progress = Decode_Progress.Decode_None
        self.PitchPID_Para = single_pid()
        self.RollPID_Para = single_pid()
        self.GyrXPID_Para = single_pid()
        self.GyrYPID_Para = single_pid()
        self.GyrZPID_Para = single_pid()

    def parse(self, bytes):
        if len(bytes) == 0:
            return False

        for i in bytes:
            if i.find("Pitch"):
                if self.decode_progress != Decode_Progress.Decode_None:
                    return False
                self.decode_progress = Decode_Progress.Decode_PitchPart

            if i.find("Roll"):
                if self.decode_progress != Decode_Progress.Decode_None:
                    return False
                self.decode_progress = Decode_Progress.Decode_RollPart

            if i.find("GyroX"):
                if self.decode_progress != Decode_Progress.Decode_None:
                    return False
                self.decode_progress = Decode_Progress.Decode_GyroXPart
            
            if i.find("GyroY"):
                if self.decode_progress != Decode_Progress.Decode_None:
                    return False
                self.decode_progress = Decode_Progress.Decode_GyroYPart

            if i.find("GyroZ"):
                if self.decode_progress != Decode_Progress.Decode_None:
                    return False
                self.decode_progress = Decode_Progress.Decode_GyroZPart

    def get(self):
        pass