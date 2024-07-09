import sys
from queue import Queue
from pymavlink import mavutil

def clear_consoel_dsp(line = 0):
    for i in range(line):
        sys.stdout.write('\x1b[1A')
        sys.stdout.write('\x1b[2K')

def mav_parse(queue):
    readable_size = queue.qsize()
    print(" ----> [ mav parse input queue size ] :", readable_size)
    clear_consoel_dsp(1)
    # for i in range(readable_size):