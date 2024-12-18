from tkinter import *
import time
import Att_CasecadePID
from Att_CasecadePID import ParaItem_Index
from pid_para import PIDItem_Index

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

        self.__para_init__()
        self.__UI_init__()

    def __para_init__(self):
        self.att_pid = Att_CasecadePID.Att_CaseCadePID(self.__port)

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

    def __UI_init__(self):
        self.__UI = Tk()
        self.__UI.title("Controller Tune")
        self.__UI.geometry("240x480")

        self.__Pitch_label = Label(self.__UI, text = "Pitch")
        self.__Pitch_P_sv = StringVar()
        self.__Pitch_I_sv = StringVar()
        self.__Pitch_D_sv = StringVar()
        self.__Pitch_P_Entry = Spinbox(self.__UI, from_ = 1, to = 10, textvariable = self.__Pitch_P_sv)
        self.__Pitch_I_Entry = Spinbox(self.__UI, from_ = 1, to = 10, textvariable = self.__Pitch_I_sv)
        self.__Pitch_D_Entry = Spinbox(self.__UI, from_ = 1, to = 10, textvariable = self.__Pitch_D_sv)
        self.__Pitch_P_sv.set(self.__pitch_p)
        self.__Pitch_I_sv.set(self.__pitch_i)
        self.__Pitch_D_sv.set(self.__pitch_d)

        self.__Roll_label = Label(self.__UI, text = "Roll")
        self.__Roll_P_sv = StringVar()
        self.__Roll_I_sv = StringVar()
        self.__Roll_D_sv = StringVar()
        self.__Roll_P_Entry = Spinbox(self.__UI, from_ = 1, to = 10, textvariable = self.__Roll_P_sv)
        self.__Roll_I_Entry = Spinbox(self.__UI, from_ = 1, to = 10, textvariable = self.__Roll_I_sv)
        self.__Roll_D_Entry = Spinbox(self.__UI, from_ = 1, to = 10, textvariable = self.__Roll_D_sv)
        self.__Roll_P_sv.set(self.__roll_p)
        self.__Roll_I_sv.set(self.__roll_i)
        self.__Roll_D_sv.set(self.__roll_d)

        self.__GyroX_label = Label(self.__UI, text = "GyroX")
        self.__GX_P_sv = StringVar()
        self.__GX_I_sv = StringVar()
        self.__GX_D_sv = StringVar()
        self.__GX_P_Entry = Spinbox(self.__UI, from_ = 1, to = 10, textvariable = self.__GX_P_sv)
        self.__GX_I_Entry = Spinbox(self.__UI, from_ = 1, to = 10, textvariable = self.__GX_I_sv)
        self.__GX_D_Entry = Spinbox(self.__UI, from_ = 1, to = 10, textvariable = self.__GX_D_sv)
        self.__GX_P_sv.set(self.__gX_p)
        self.__GX_I_sv.set(self.__gX_i)
        self.__GX_D_sv.set(self.__gX_d)

        self.__GyroY_label = Label(self.__UI, text = "GyroY")
        self.__GY_P_sv = StringVar()
        self.__GY_I_sv = StringVar()
        self.__GY_D_sv = StringVar()
        self.__GY_P_Entry = Spinbox(self.__UI, from_ = 1, to = 10, textvariable = self.__GY_P_sv)
        self.__GY_I_Entry = Spinbox(self.__UI, from_ = 1, to = 10, textvariable = self.__GY_I_sv)
        self.__GY_D_Entry = Spinbox(self.__UI, from_ = 1, to = 10, textvariable = self.__GY_D_sv)
        self.__GY_P_sv.set(self.__gY_p)
        self.__GY_I_sv.set(self.__gY_i)
        self.__GY_D_sv.set(self.__gY_d)

        self.__GyroZ_label = Label(self.__UI, text = "GyroZ")
        self.__GZ_P_sv = StringVar()
        self.__GZ_I_sv = StringVar()
        self.__GZ_D_sv = StringVar()
        self.__GZ_P_Entry = Spinbox(self.__UI, from_ = 1, to = 10, textvariable = self.__GZ_P_sv)
        self.__GZ_I_Entry = Spinbox(self.__UI, from_ = 1, to = 10, textvariable = self.__GZ_I_sv)
        self.__GZ_D_Entry = Spinbox(self.__UI, from_ = 1, to = 10, textvariable = self.__GZ_D_sv)
        self.__GZ_P_sv.set(self.__gZ_p)
        self.__GZ_I_sv.set(self.__gZ_i)
        self.__GZ_D_sv.set(self.__gZ_d)

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

    def __spinbox_check(self, spinbox_in, ref):
        try:
            tmp = float(spinbox_in)
            if tmp < 0:
                print("[ Error input ]")
                return ref
            else:
                return tmp
        except ValueError:
            print("[ Error input ]")
            return ref

    def __Send_Release(self):
        CmdList = [['tune_att_pid 0 '],
                   ['tune_att_pid 1 '], 
                   ['tune_att_pid 2 '], 
                   ['tune_att_pid 3 '], 
                   ['tune_att_pid 4 ']]
        
        # get data in spinbox
        # get pitch pid parameter
        self.__pitch_p = self.__spinbox_check(self.__Pitch_P_sv.get(), self.__pitch_p)
        self.__pitch_i = self.__spinbox_check(self.__Pitch_I_sv.get(), self.__pitch_i)
        self.__pitch_d = self.__spinbox_check(self.__Pitch_D_sv.get(), self.__pitch_d)
        
        # get roll  pid parameter
        self.__roll_p = self.__spinbox_check(self.__Roll_P_sv.get(), self.__roll_p)
        self.__roll_i = self.__spinbox_check(self.__Roll_I_sv.get(), self.__roll_i)
        self.__roll_d = self.__spinbox_check(self.__Roll_D_sv.get(), self.__roll_d)
                                                         
        # get gyroX pid parameter
        self.__gX_p = self.__spinbox_check(self.__GX_P_sv.get(), self.__gX_p)
        self.__gX_i = self.__spinbox_check(self.__GX_I_sv.get(), self.__gX_i)
        self.__gX_d = self.__spinbox_check(self.__GX_D_sv.get(), self.__gX_d)

        # get gyroY pid parameter
        self.__gY_p = self.__spinbox_check(self.__GY_P_sv.get(), self.__gY_p)
        self.__gY_i = self.__spinbox_check(self.__GY_I_sv.get(), self.__gY_i)
        self.__gY_d = self.__spinbox_check(self.__GY_D_sv.get(), self.__gY_d)
 
        # get gyroZ pid parameter
        self.__gZ_p = self.__spinbox_check(self.__GZ_P_sv.get(), self.__gZ_p)
        self.__gZ_i = self.__spinbox_check(self.__GZ_I_sv.get(), self.__gZ_i)
        self.__gZ_d = self.__spinbox_check(self.__GZ_D_sv.get(), self.__gZ_d)

        # for i in CmdList:
        #     time.sleep(0.5)

    def __Get_Release(self):
        pass

    def Tune(self):
        self.__UI.mainloop()