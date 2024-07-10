import os
import sys
import serial
import serial.tools.list_ports
import serial.tools.list_ports_common
from time import sleep
from mav_parse import H7FC_Obj as Drone

def Kb(size = 0):
    return size * 1024 * 1024

def Mb(size = 0):
    return size * 1024 * Kb(1)

def clear_consoel_dsp(line = 0):
    for i in range(line):
        sys.stdout.write('\x1b[1A')
        sys.stdout.write('\x1b[2K')

research_cnt = 0

while (True):
    avaliable_port = list(serial.tools.list_ports.comports())
    research_cnt = research_cnt + 1
    os.system('clear')
    print("[ detected port number ] -------- ", len(avaliable_port))

    FC_Found = False

    if len(avaliable_port):
        for port_info in avaliable_port:
            if (port_info.description == 'H7FC') and (port_info.manufacturer == '8_B!T0'):
                FC_Found = True
                print("\t[ Flight Controller Found ]")
                print("\t[ --- PORT INFO --- ]")
                print("\t[ name          ]: ", port_info.name)
                print("\t[ device        ]: ", port_info.device)
                print("\t[ pid           ]: ", port_info.pid)
                print("\t[ serial number ]: ", port_info.serial_number)
                print("\t[ description   ]: ", port_info.description)
                print("\t[ product       ]: ", port_info.product)
                print("\t[ manufacturer  ]: ", port_info.manufacturer)
                print("\r\n")
                break

    # if found flight controller is attach
    # then open seleceted port
    if FC_Found:
        FC_port = serial.Serial(port_info.device, 460800, 5)
        
        if FC_port.is_open:
            print("[ Flight Controller Port Open Successed ]\r\n")
            H7FC = Drone(port_info.device)

            # do mavlink and other frame parse
            # communicate with the flight controller
            rec_size = 0
            while FC_port.is_open:
                # # have data receive
                rec_size = FC_port.in_waiting
                if rec_size > 0:
                    print('[ receive size ]\t', rec_size)
                    clear_consoel_dsp(1)
                    H7FC.parse()
                sleep(0.2)

            print("[ Flight Controller is disconnected ]")

        else :
            print("\t[ Flight Controller Port Open Failed ]")
    else :
        print("\t[ Flight Controller Not Found ]")
        print("\t[ Research ] ----------------- ", research_cnt)
        sleep(1)
