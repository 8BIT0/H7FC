from enum import Enum

class PID_Parse_State(Enum):
    Parse_None = 0
    Parsing = 1
    Parse_Error = 2
    Parse_Fin = 3

class PIDItem_Index(Enum):
    PIDIndex_P = 0
    PIDIndex_I = 1
    PIDIndex_D = 2

class PID_Param:
    def __init__(self):
        self.__parse_num = 0
        self.__parse_state = PID_Parse_State.Parse_None
        self.__PID_Dict = {'P':0.0, 'I':0.0, 'D':0.0}

    def clear(self):
        self.__PID_Dict['P'] = 0.0
        self.__PID_Dict['I'] = 0.0
        self.__PID_Dict['D'] = 0.0

    def parse(self, bytes):
        str_data = None
        match = False
        if len(bytes):
            self.__parse_state = PID_Parse_State.Parsing
            if bytes.decode("ASCII").find("P: ") != -1:
                str_data = bytes.decode("ASCII").rstrip("\r\n").split("P: ")[1]
                self.__PID_Dict['P'] = float(str_data)
                print("[ Parsing P ]", self.__PID_Dict['P'])
                match = True
            elif bytes.decode("ASCII").find("I: ") != -1:
                str_data = bytes.decode("ASCII").rstrip("\r\n").split("I: ")[1]
                self.__PID_Dict['I'] = float(str_data)
                print("[ Parsing I ]", self.__PID_Dict['I'])
                match = True
            elif bytes.decode("ASCII").find("D: ") != -1:
                str_data = bytes.decode("ASCII").rstrip("\r\n").split("D: ")[1]
                self.__PID_Dict['D'] = float(str_data)
                print("[ Parsing D ]", self.__PID_Dict['D'])
                match = True
                self.__parse_num += 1
                print("[ Parsing finish ]\r\n")
                self.__parse_state = PID_Parse_State.Parse_Fin
        
        if not match:
            self.__parse_state = PID_Parse_State.Parse_Error

        return self.__parse_state
    
    def format_str(self):
        if self.__parse_num == 0:
            return ""
        
        format =  "P = " + str(self.__PID_Dict["P"]) + "\t\t"
        format += "I = " + str(self.__PID_Dict["I"]) + "\t\t"
        format += "D = " + str(self.__PID_Dict["D"]) + "\r\n"
        return format
    
    def get_val(self):
        return [self.__PID_Dict["P"], self.__PID_Dict["I"], self.__PID_Dict["D"]]