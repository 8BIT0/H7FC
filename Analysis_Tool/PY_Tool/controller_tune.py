from tkinter import *
import time
import Att_CasecadePID
from Att_CasecadePID import ParaItem_Index
from pid_para import PIDItem_Index
from enum import Enum

class TuneSet_Ack(Enum):
    Tune_Wait = 0
    Tune_Failed = 1
    Tune_Done = 2
    Tune_TimeOut = 3

class Controller_Tune_AttPID:
    def __init__(self, port):
        self.__port = port

        self.__pitch_p = 0.0
        self.__pitch_i = 0.0
        self.__pitch_d = 0.0

        self.__roll_p = 0.0
        self.__roll_i = 0.0
        self.__roll_d = 0.0

        self.__gX_p = 0.0
        self.__gX_i = 0.0
        self.__gX_d = 0.0
        
        self.__gY_p = 0.0
        self.__gY_i = 0.0
        self.__gY_d = 0.0

        self.__gZ_p = 0.0
        self.__gZ_i = 0.0
        self.__gZ_d = 0.0
        self.__P_range = (0.01, 20)
        self.__I_range = (0, 20)
        self.__D_range = (0, 20)

        self.__para_init__()
        self.__UI_init__()

    def __para_init__(self):
        self.att_pid = Att_CasecadePID.Att_CaseCadePID(self.__port)
        self.__get_para()

    def __UI_init__(self):
        self.__UI = Tk()
        self.__UI.title("Controller Tune")
        self.__UI.geometry("240x480")

        self.__Pitch_label = Label(self.__UI, text = "Pitch")
        self.__Pitch_P_sv = StringVar()
        self.__Pitch_I_sv = StringVar()
        self.__Pitch_D_sv = StringVar()
        self.__Pitch_P_Entry = Spinbox(self.__UI, from_ = 1, to = 20, textvariable = self.__Pitch_P_sv)
        self.__Pitch_I_Entry = Spinbox(self.__UI, from_ = 1, to = 20, textvariable = self.__Pitch_I_sv)
        self.__Pitch_D_Entry = Spinbox(self.__UI, from_ = 1, to = 20, textvariable = self.__Pitch_D_sv)

        self.__Roll_label = Label(self.__UI, text = "Roll")
        self.__Roll_P_sv = StringVar()
        self.__Roll_I_sv = StringVar()
        self.__Roll_D_sv = StringVar()
        self.__Roll_P_Entry = Spinbox(self.__UI, from_ = 1, to = 20, textvariable = self.__Roll_P_sv)
        self.__Roll_I_Entry = Spinbox(self.__UI, from_ = 1, to = 20, textvariable = self.__Roll_I_sv)
        self.__Roll_D_Entry = Spinbox(self.__UI, from_ = 1, to = 20, textvariable = self.__Roll_D_sv)

        self.__GyroX_label = Label(self.__UI, text = "GyroX")
        self.__GX_P_sv = StringVar()
        self.__GX_I_sv = StringVar()
        self.__GX_D_sv = StringVar()
        self.__GX_P_Entry = Spinbox(self.__UI, from_ = 1, to = 20, textvariable = self.__GX_P_sv)
        self.__GX_I_Entry = Spinbox(self.__UI, from_ = 1, to = 20, textvariable = self.__GX_I_sv)
        self.__GX_D_Entry = Spinbox(self.__UI, from_ = 1, to = 20, textvariable = self.__GX_D_sv)

        self.__GyroY_label = Label(self.__UI, text = "GyroY")
        self.__GY_P_sv = StringVar()
        self.__GY_I_sv = StringVar()
        self.__GY_D_sv = StringVar()
        self.__GY_P_Entry = Spinbox(self.__UI, from_ = 1, to = 20, textvariable = self.__GY_P_sv)
        self.__GY_I_Entry = Spinbox(self.__UI, from_ = 1, to = 20, textvariable = self.__GY_I_sv)
        self.__GY_D_Entry = Spinbox(self.__UI, from_ = 1, to = 20, textvariable = self.__GY_D_sv)

        self.__GyroZ_label = Label(self.__UI, text = "GyroZ")
        self.__GZ_P_sv = StringVar()
        self.__GZ_I_sv = StringVar()
        self.__GZ_D_sv = StringVar()
        self.__GZ_P_Entry = Spinbox(self.__UI, from_ = 1, to = 20, textvariable = self.__GZ_P_sv)
        self.__GZ_I_Entry = Spinbox(self.__UI, from_ = 1, to = 20, textvariable = self.__GZ_I_sv)
        self.__GZ_D_Entry = Spinbox(self.__UI, from_ = 1, to = 20, textvariable = self.__GZ_D_sv)

        self.__send_button = Button(self.__UI, text = "-- send --", command = self.__Send_Release)
        self.__get_button = Button(self.__UI, text = "--- get --", command = self.__Get_Release)
        
        self.__Pitch_label.pack()
        self.__Pitch_P_Entry.pack()
        self.__Pitch_I_Entry.pack()
        self.__Pitch_D_Entry.pack()

        self.__Roll_label.pack()
        self.__Roll_P_Entry.pack()
        self.__Roll_I_Entry.pack()
        self.__Roll_D_Entry.pack()

        self.__GyroX_label.pack()
        self.__GX_P_Entry.pack()
        self.__GX_I_Entry.pack()
        self.__GX_D_Entry.pack()

        self.__GyroY_label.pack()
        self.__GY_P_Entry.pack()
        self.__GY_I_Entry.pack()
        self.__GY_D_Entry.pack()

        self.__GyroZ_label.pack()
        self.__GZ_P_Entry.pack()
        self.__GZ_I_Entry.pack()
        self.__GZ_D_Entry.pack()

        self.__send_button.pack()
        self.__get_button.pack()
        self.__para_dsp()

    def __sys_ms(self):
        return int(time.time() * 1000)

    def __check_set_ack(self, bytes):
        if len(bytes):
            if bytes.decode("ASCII").find("[ ---- parameter saved ---- ]") != -1:
                return TuneSet_Ack.Tune_Done
            elif bytes.decode("ASCII").find("[ ---- parameter save failed ---- ]") != -1 or \
                 bytes.decode("ASCII").find("[ ---- parameter set failed ---- ]") != -1:
                 return TuneSet_Ack.Tune_Failed
            return TuneSet_Ack.Tune_Wait
        return TuneSet_Ack.Tune_Wait

    def __spinbox_check(self, spinbox_in, range, ref):
        try:
            tmp = float(spinbox_in)
            if tmp < 0:
                print("[ Error input ]")
                return ref
            else:
                if tmp > range[1] or tmp < range[0]:
                    return ref
                return tmp
        except ValueError:
            print("[ Error input ]")
            return ref

    def __format_str(self, v_p, v_i, v_d):
        format =  "P" + str(v_p) + " "
        format += "I" + str(v_i) + " "
        format += "D" + str(v_d) + "\r\n"
        return format

    def __Send_Release(self):
        CmdList = ['tune_att_pid 0 ',
                   'tune_att_pid 1 ', 
                   'tune_att_pid 2 ', 
                   'tune_att_pid 3 ', 
                   'tune_att_pid 4 ']
        
        # get data in spinbox
        # get pitch pid parameter
        self.__pitch_p = self.__spinbox_check(self.__Pitch_P_sv.get(), self.__P_range, self.__pitch_p)
        self.__pitch_i = self.__spinbox_check(self.__Pitch_I_sv.get(), self.__I_range, self.__pitch_i)
        self.__pitch_d = self.__spinbox_check(self.__Pitch_D_sv.get(), self.__D_range, self.__pitch_d)
        CmdList[ParaItem_Index.Item_Pitch.value] = CmdList[ParaItem_Index.Item_Pitch.value] + self.__format_str(self.__pitch_p, self.__pitch_i, self.__pitch_d)

        # get roll  pid parameter
        self.__roll_p = self.__spinbox_check(self.__Roll_P_sv.get(), self.__P_range, self.__roll_p)
        self.__roll_i = self.__spinbox_check(self.__Roll_I_sv.get(), self.__I_range, self.__roll_i)
        self.__roll_d = self.__spinbox_check(self.__Roll_D_sv.get(), self.__D_range, self.__roll_d)
        CmdList[ParaItem_Index.Item_Roll.value] = CmdList[ParaItem_Index.Item_Roll.value] + self.__format_str(self.__roll_p, self.__roll_i, self.__roll_d)
        
        # get gyroX pid parameter
        self.__gX_p = self.__spinbox_check(self.__GX_P_sv.get(), self.__P_range, self.__gX_p)
        self.__gX_i = self.__spinbox_check(self.__GX_I_sv.get(), self.__I_range, self.__gX_i)
        self.__gX_d = self.__spinbox_check(self.__GX_D_sv.get(), self.__D_range, self.__gX_d)
        CmdList[ParaItem_Index.Item_GyroX.value] = CmdList[ParaItem_Index.Item_GyroX.value] + self.__format_str(self.__gX_p, self.__gX_i, self.__gX_d)

        # get gyroY pid parameter
        self.__gY_p = self.__spinbox_check(self.__GY_P_sv.get(), self.__P_range, self.__gY_p)
        self.__gY_i = self.__spinbox_check(self.__GY_I_sv.get(), self.__I_range, self.__gY_i)
        self.__gY_d = self.__spinbox_check(self.__GY_D_sv.get(), self.__D_range, self.__gY_d)
        CmdList[ParaItem_Index.Item_GyroY.value] = CmdList[ParaItem_Index.Item_GyroY.value] + self.__format_str(self.__gY_p, self.__gY_i, self.__gY_d)
 
        # get gyroZ pid parameter
        self.__gZ_p = self.__spinbox_check(self.__GZ_P_sv.get(), self.__P_range, self.__gZ_p)
        self.__gZ_i = self.__spinbox_check(self.__GZ_I_sv.get(), self.__I_range, self.__gZ_i)
        self.__gZ_d = self.__spinbox_check(self.__GZ_D_sv.get(), self.__D_range, self.__gZ_d)
        CmdList[ParaItem_Index.Item_GyroZ.value] = CmdList[ParaItem_Index.Item_GyroZ.value] + self.__format_str(self.__gZ_p, self.__gZ_i, self.__gZ_d)

        print("[ Send attitdue controller parameter to drone ]")

        index = 0
        sys_time = self.__sys_ms()
        while True:
            print(CmdList[index])
            self.__port.write(CmdList[index].encode("ASCII"))
            while True:
                ack_str = self.__port.readline()
                print(ack_str)
                state = self.__check_set_ack(ack_str)
                if state == TuneSet_Ack.Tune_Done:
                    print("[ controller parameter set done ]")
                    # update sys time
                    sys_time = self.__sys_ms()
                    index += 1
                    break
                elif state == TuneSet_Ack.Tune_Failed:
                    print("[ controller parameter set failed ]")
                    return
                elif state == TuneSet_Ack.Tune_Wait:
                    # check ack time out
                    if self.__sys_ms() - sys_time >= 2000:
                        print("[ controller parameter set time out ]")
                        return
                time.sleep(0.05)

            if index > ParaItem_Index.Item_GyroZ.value:
                print("[ controller parameter set finish ]")
                return

    def __clear_dsp(self):
        self.__Pitch_P_sv.set(0.0)
        self.__Pitch_I_sv.set(0.0)
        self.__Pitch_D_sv.set(0.0)

        self.__Roll_P_sv.set(0.0)
        self.__Roll_I_sv.set(0.0)
        self.__Roll_D_sv.set(0.0)

        self.__GX_P_sv.set(0.0)
        self.__GX_I_sv.set(0.0)
        self.__GX_D_sv.set(0.0)

        self.__GY_P_sv.set(0.0)
        self.__GY_I_sv.set(0.0)
        self.__GY_D_sv.set(0.0)
        
        self.__GZ_P_sv.set(0.0)
        self.__GZ_I_sv.set(0.0)
        self.__GZ_D_sv.set(0.0)

    def __get_para(self):
        # require controller parameter
        if not self.att_pid.parse_para():
            print("[ Attitude controller parameter get failed ]")
        else:
            # set attitude controller parameter
            para_list = self.att_pid.get_value()
            self.__pitch_p = para_list[ParaItem_Index.Item_Pitch.value][PIDItem_Index.PIDIndex_P.value]
            self.__pitch_i = para_list[ParaItem_Index.Item_Pitch.value][PIDItem_Index.PIDIndex_I.value]
            self.__pitch_d = para_list[ParaItem_Index.Item_Pitch.value][PIDItem_Index.PIDIndex_D.value]
            
            self.__roll_p = para_list[ParaItem_Index.Item_Roll.value][PIDItem_Index.PIDIndex_P.value]
            self.__roll_i = para_list[ParaItem_Index.Item_Roll.value][PIDItem_Index.PIDIndex_I.value]
            self.__roll_d = para_list[ParaItem_Index.Item_Roll.value][PIDItem_Index.PIDIndex_D.value]
            
            self.__gX_p = para_list[ParaItem_Index.Item_GyroX.value][PIDItem_Index.PIDIndex_P.value]
            self.__gX_i = para_list[ParaItem_Index.Item_GyroX.value][PIDItem_Index.PIDIndex_I.value]
            self.__gX_d = para_list[ParaItem_Index.Item_GyroX.value][PIDItem_Index.PIDIndex_D.value]
            
            self.__gY_p = para_list[ParaItem_Index.Item_GyroY.value][PIDItem_Index.PIDIndex_P.value]
            self.__gY_i = para_list[ParaItem_Index.Item_GyroY.value][PIDItem_Index.PIDIndex_I.value]
            self.__gY_d = para_list[ParaItem_Index.Item_GyroY.value][PIDItem_Index.PIDIndex_D.value]
            
            self.__gZ_p = para_list[ParaItem_Index.Item_GyroZ.value][PIDItem_Index.PIDIndex_P.value]
            self.__gZ_i = para_list[ParaItem_Index.Item_GyroZ.value][PIDItem_Index.PIDIndex_I.value]
            self.__gZ_d = para_list[ParaItem_Index.Item_GyroZ.value][PIDItem_Index.PIDIndex_D.value]

    def __para_dsp(self):
        self.__Pitch_P_sv.set(self.__pitch_p)
        self.__Pitch_I_sv.set(self.__pitch_i)
        self.__Pitch_D_sv.set(self.__pitch_d)
        self.__Roll_P_sv.set(self.__roll_p)
        self.__Roll_I_sv.set(self.__roll_i)
        self.__Roll_D_sv.set(self.__roll_d)
        self.__GX_P_sv.set(self.__gX_p)
        self.__GX_I_sv.set(self.__gX_i)
        self.__GX_D_sv.set(self.__gX_d)
        self.__GY_P_sv.set(self.__gY_p)
        self.__GY_I_sv.set(self.__gY_i)
        self.__GY_D_sv.set(self.__gY_d)
        self.__GZ_P_sv.set(self.__gZ_p)
        self.__GZ_I_sv.set(self.__gZ_i)
        self.__GZ_D_sv.set(self.__gZ_d)

    def __Get_Release(self):
        self.__clear_dsp()
        time.sleep(0.1)
        self.__get_para()
        self.__para_dsp()

    def Tune(self):
        self.__UI.mainloop()