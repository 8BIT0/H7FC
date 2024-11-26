class PID_Param:
    def __init__(self):
        self.decodeing = False
        self.PID_Dict = {'P':0.0, 'I':0.0, 'D':0.0}

    def __set(self, P, I, D):
        self.PID_Dict['P'] = P
        self.PID_Dict['I'] = I
        self.PID_Dict['D'] = D

    def parse(self, bytes):
        if len(bytes):
            self.decodeing = True

        return self.PID_Dict