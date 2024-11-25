class PID_Param:
    def __init__(self):
        self.PID_Dict = {'P':0.0, 'I':0.0, 'D':0.0}

    def Set(self, P, I, D):
        self.PID_Dict['P'] = P
        self.PID_Dict['I'] = I
        self.PID_Dict['D'] = D

    def Get(self):
        return self.PID_Dict