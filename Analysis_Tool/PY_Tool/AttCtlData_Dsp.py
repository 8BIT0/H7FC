import os
from enum import Enum
import matplotlib.pyplot as plt

class Dsp_Type(Enum):
    Dsp_Angular     = 1
    Dsp_Attitude    = 2

class AngularControlData(object):
    def __init__(self):
        self.AngularDict = {'T':[0], 'Throttle':[0], 'e_GX':[0.0], 'e_GY':[0.0], 'e_GZ':[0.0], 'm_GX':[0.0], 'm_GY':[0.0], 'm_GZ':[0.0], 'd_GX':[0.0], 'd_GY':[0.0], 'd_GZ':[0.0]}
    
    def append(self, time, throttle, exp_GX, exp_GY, exp_GZ, mea_GX, mea_GY, mea_GZ):
        self.AngularDict['T'].append(time)
        self.AngularDict['Throttle'].append(throttle)
        self.AngularDict['e_GX'].append(exp_GX)
        self.AngularDict['e_GY'].append(exp_GY)
        self.AngularDict['e_GZ'].append(exp_GZ)
        self.AngularDict['m_GX'].append(mea_GX)
        self.AngularDict['m_GY'].append(mea_GY)
        self.AngularDict['m_GZ'].append(mea_GZ)
        self.AngularDict['d_GX'].append(exp_GX - mea_GX)
        self.AngularDict['d_GY'].append(exp_GY - mea_GY)
        self.AngularDict['d_GZ'].append(exp_GZ - mea_GZ)

    def get(self):
        return self.AngularDict

class AttitudeControlData(object):
    def __init__(self):
        self.AttitudeDict = {'T':[0], 'Throttle':[0], 'e_P':[0.0], 'e_R':[0.0], 'm_P':[0.0], 'm_R':[0.0], 'd_P':[0.0], 'd_R':[0.0]}

    def append(self, time, throttle, exp_pitch, exp_roll, mea_pitch, mea_roll):
        self.AttitudeDict['T'].append(time)
        self.AttitudeDict['Throttle'].append(throttle)
        self.AttitudeDict['e_P'].append(exp_pitch)
        self.AttitudeDict['e_R'].append(exp_roll)
        self.AttitudeDict['m_P'].append(mea_pitch)
        self.AttitudeDict['m_R'].append(mea_roll)
        self.AttitudeDict['d_P'].append(exp_pitch - mea_pitch)
        self.AttitudeDict['d_R'].append(exp_roll - mea_roll)

    def get(self):
        return self.AttitudeDict

class ControlData_Display(object):
    def __init__(self):
        self.folder_dir = os.path.dirname(os.path.abspath(__file__))
        self.angular_file_list = [f for f in os.listdir(self.folder_dir) if f.endswith('.txt') and f.startswith('log_AngularPID')]
        print('angular log file number', len(self.angular_file_list))

        self.attitude_file_list = [f for f in os.listdir(self.folder_dir) if f.endswith('.txt') and f.startswith('log_AttitudePID')]

    def dsp_angular_ctl(self):
        for file_name in self.angular_file_list:
            print('file name', file_name)
            ang_dict_list = self.__load_file(Dsp_Type.Dsp_Angular, file_name)
            plt.figure()
            
            plt.subplot(3, 1, 1)
            plt.plot(ang_dict_list['e_GX'], label = 'e_GX')
            plt.plot(ang_dict_list['m_GX'], label = 'm_GX')
            plt.plot(ang_dict_list['Throttle'])
            plt.legend()
            
            plt.subplot(3, 1, 2)
            plt.plot(ang_dict_list['e_GY'], label = 'e_GY')
            plt.plot(ang_dict_list['m_GY'], label = 'm_GY')
            plt.plot(ang_dict_list['Throttle'])
            plt.legend()
            
            plt.subplot(3, 1, 3)
            plt.plot(ang_dict_list['e_GZ'], label = 'e_GZ')
            plt.plot(ang_dict_list['m_GZ'], label = 'm_GZ')
            plt.plot(ang_dict_list['Throttle'])
            plt.legend()
        plt.show()

    def dsp_attitude_ctl(self):
        pass

    def __load_file(self, type, file_name):
        file = open(file_name, 'r')
        eof = file.seek(0, 2)
        file.seek(0, 0)
        para_end = False
        DataDict_List = None
        while True:
            if file.tell() >= eof:
                file.close()
                return DataDict_List.get()

            line = file.readline()
            if not para_end and (line.find('-----------------------------------------------------------------------------') != -1):
                para_end = True
                if type == Dsp_Type.Dsp_Angular:
                    DataDict_List = AngularControlData()
                elif type == Dsp_Type.Dsp_Attitude:
                    DataDict_List = AttitudeControlData()
                continue
            else:
                # pase controller parameter
                pass
 
            data_list = line.split(' ')
            if para_end and len(data_list):
                if type == Dsp_Type.Dsp_Angular:
                    DataDict_List.append(int(data_list[0]), float(data_list[1]), 
                                         float(data_list[2]), float(data_list[3]), float(data_list[4]), 
                                         float(data_list[5]), float(data_list[6]), float(data_list[7]))
                elif type == Dsp_Type.Dsp_Attitude:
                    pass

dsp = ControlData_Display()
dsp.dsp_angular_ctl()
