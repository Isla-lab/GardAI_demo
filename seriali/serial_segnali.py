import serial
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import time
import numpy as np
import math as math


from classes.classes_externals import *
from functions.Serials_functions.functions_external import *

# Reads telemetry lines from the PIC/H2O/sonar serial connector (one message
# per line, e.g. "$H2O_,temp,ph,tds,oxygen,*crc") and dispatches each one to
# handle_command, which fills the shared signal_data object.
def read_signal(ser,signal_data):

    while True:
        if 1==1: #not stop_event.is_set():
            value =ser.readline().decode("utf-8")
            value_splitted = value.split(",")
            cmd_type = value_splitted[0]
            print(handle_command(cmd_type,signal_data,value_splitted))
