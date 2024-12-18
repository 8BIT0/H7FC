#!/usr/bin/env python3
import os
import sys
import serial
import serial.tools.list_ports
import serial.tools.list_ports_common
from time import sleep
from cli_control import CLI_Ctl as CLI
from cli_control import CLI_State

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
    
    # if your operation system is windows make sure you install AT32 and STM32 VCP driver already
    if len(avaliable_port):
        for port_info in avaliable_port:
            print(port_info)
            # if (port_info.description.find('H7FC') >= 0) or (port_info.manufacturer.find('8_B!T0') >= 0):
            if port_info.description.find('H7FC') >= 0:
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

            # communicate with the flight controller
            Cli_Ctl = CLI(FC_port)
            state = Cli_Ctl.Into_CLI_Mode()
                
            while state == CLI_State.CLI_No_Error:
                tool_cli = input()
                if tool_cli.find("quit") != -1:
                    Cli_Ctl.Quit_CLI()
                    FC_port.close()
                    break
                elif tool_cli.find("bb") != -1:
                    Cli_Ctl.Get_Blackbox_Data()
                elif tool_cli.find("tune") != -1:
                    Cli_Ctl.Tune_Controller()
                else:
                    print("[ Unknown cli input ]")

                sleep(0.2)

            FC_port.close()
            print("[ Flight Controller is disconnected ]")
        else :
            print("\t[ Flight Controller Port Open Failed ]")
    else :
        print("\t[ Flight Controller Not Found ]")
        print("\t[ Research ] ----------------- ", research_cnt)
        sleep(1)
