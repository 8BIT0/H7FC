from enum import Enum

class PID_Parse_State(Enum):
    Parse_None = 0
    Parsing = 1
    Parse_Error = 2
    Parse_Fin = 3

class PID_Param:
    def __init__(self):
        self.__parse_num = 0
        self.__parse_state = PID_Parse_State.Parse_None
        self.PID_Dict = {'P':0.0, 'I':0.0, 'D':0.0}

    def parse(self, bytes):
        str_data = None
        match = False
        if len(bytes):
            self.__parse_state = PID_Parse_State.Parsing
            if bytes.decode("ASCII").find("P: ") != -1:
                str_data = bytes.decode("ASCII").rstrip("\r\n").split("P: ")[1]
                self.PID_Dict['P'] = float(str_data)
                print("[ Parsing P ]", self.PID_Dict['P'])
                match = True
            elif bytes.decode("ASCII").find("I: ") != -1:
                str_data = bytes.decode("ASCII").rstrip("\r\n").split("I: ")[1]
                self.PID_Dict['I'] = float(str_data)
                print("[ Parsing I ]", self.PID_Dict['I'])
                match = True
            elif bytes.decode("ASCII").find("D: ") != -1:
                str_data = bytes.decode("ASCII").rstrip("\r\n").split("D: ")[1]
                self.PID_Dict['D'] = float(str_data)
                print("[ Parsing D ]", self.PID_Dict['D'])
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
        
        str =  "P =" + str(self.PID_Dict["P"]) + "\t\t"
        str += "I =" + str(self.PID_Dict["I"]) + "\t\t"
        str += "D =" + str(self.PID_Dict["D"]) + "\r\n"
        return str