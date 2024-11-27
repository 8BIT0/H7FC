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
            if bytes.find("P: "):
                match = True
            elif bytes.find("I: "):
                match = True
            elif bytes.find("D: "):
                match = True
                self.__parse_state = PID_Parse_State.Parse_Fin
        
        if not match:
            self.__parse_state = PID_Parse_State.Parse_Error

        return self.__parse_state
    
    def get(self):
        pass