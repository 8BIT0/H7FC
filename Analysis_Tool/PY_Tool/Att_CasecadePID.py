from pid_para import PID_Param as single_pid

class Att_CaseCadePID:
    def __init__(self):
        self.PitchPID_Para = single_pid()
        self.RollPID_Para = single_pid()
        self.GyrXPID_Para = single_pid()
        self.GyrYPID_Para = single_pid()
        self.GyrZPID_Para = single_pid()

    def parse(self, bytes):
        if len(bytes):
            pass

    def get(self):
        pass