from enum import Enum

class PID_Parse_State(Enum):
    Parse_None = 0
    Parsing = 1
    Parse_Error = 2
    Parse_Fin = 3

class PID_Param:
    def __init__(self):
        self.__parse_state = PID_Parse_State.Parse_None
        self.PID_Dict = {'P':0.0, 'I':0.0, 'D':0.0}

    def parse(self, bytes):
        data = 0
        match = False
        if len(bytes):
            self.__parse_state = PID_Parse_State.Parsing
            if bytes.decode("ASCII").find("P: ") != -1:
                print("[ Parsing P ]")
                match = True
            elif bytes.decode("ASCII").find("I: ") != -1:
                print("[ Parsing I ]")
                match = True
            elif bytes.decode("ASCII").find("D: ") != -1:
                print("[ Parsing D ]")
                match = True
                print("[ Parsing finish ]\r\n")
                self.__parse_state = PID_Parse_State.Parse_Fin
        
        if not match:
            self.__parse_state = PID_Parse_State.Parse_Error

        return self.__parse_state
    
    def get(self):
        pass