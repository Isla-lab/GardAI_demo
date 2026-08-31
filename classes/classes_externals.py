#import serial
import csv
import threading
import math
import subprocess
import time
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import functions.Serials_functions.hex_functions as hexconv

from functions.Serials_functions.functions_external import *


# Decodes the u-blox UBX_NAV_RELPOSNED message (dual-antenna GPS heading).
# See Documentation/Barca-2024.pdf and the u-blox ZED-F9H interface description
# (UBX-19030118, pg. 131) for the wire format this mirrors field-by-field.
class DoppiaAntenna:
    def __init__(self):
        self.version = None
        self.reserved0 = None
        self.refStationId = None
        self.iTOW = None
        self.relPosN = None
        self.relPosE = None
        self.relPosD = None
        self.relPosLength = None
        self.relPosHeading = None
        self.reserved1 = None
        self.relPosHPN = None
        self.relPosHPE = None
        self.relPosHPD = None
        self.relPosHPLength = None
        self.accN = None
        self.accE = None
        self.accD = None
        self.accLength = None
        self.accHeading = None
        self.reserved2 = None
        self.flags = None
        
    def update_values(self, buffer):
        self.version = hexconv.hex_to_u1(buffer[0:2])
        self.reserved0 = hexconv.hex_to_u1(buffer[2:4])
        self.refStationId = hexconv.hex_to_u2(buffer[4:8])
        self.iTOW = hexconv.hex_to_u4(buffer[8:16])
        self.relPosN = hexconv.hex_to_i4(buffer[16:24])
        self.relPosE = hexconv.hex_to_i4(buffer[24:32])
        self.relPosD = hexconv.hex_to_i4(buffer[32:40])
        self.relPosLength = hexconv.hex_to_i4(buffer[40:48])
        self.relPosHeading = hexconv.hex_to_i4(buffer[48:56])
        self.reserved1 = hexconv.hex_to_u4(buffer[56:64])
        self.relPosHPN = hexconv.hex_to_i1(buffer[64:66])
        self.relPosHPE = hexconv.hex_to_i1(buffer[66:68])
        self.relPosHPD = hexconv.hex_to_i1(buffer[68:70])
        self.relPosHPLength = hexconv.hex_to_u1(buffer[70:72])
        self.accN = hexconv.hex_to_u4(buffer[72:80])
        self.accE = hexconv.hex_to_u4(buffer[80:88])
        self.accD = hexconv.hex_to_u4(buffer[88:96])
        self.accLength = hexconv.hex_to_u4(buffer[96:104])
        self.accHeading = hexconv.hex_to_u4(buffer[104:112])
        self.reserved2 = hexconv.hex_to_u4(buffer[112:120])
        self.flags = hexconv.hex_to_x4(buffer[120:128])
    
    def __str__(self):
        return (f"Version: {self.version}\n"
                f"Reserved0: {self.reserved0}\n"
                f"Reference Station ID: {self.refStationId}\n"
                f"iTOW: {self.iTOW}\n"
                f"Relative Position North: {self.relPosN}\n"
                f"Relative Position East: {self.relPosE}\n"
                f"Relative Position Down: {self.relPosD}\n"
                f"Relative Position Length: {self.relPosLength}\n"
                f"Relative Position Heading: {self.relPosHeading}\n"
                f"Reserved1: {self.reserved1}\n"
                f"Relative Position HP North: {self.relPosHPN}\n"
                f"Relative Position HP East: {self.relPosHPE}\n"
                f"Relative Position HP Down: {self.relPosHPD}\n"
                f"Relative Position HP Length: {self.relPosHPLength}\n"
                f"Acceleration North: {self.accN}\n"
                f"Acceleration East: {self.accE}\n"
                f"Acceleration Down: {self.accD}\n"
                f"Acceleration Length: {self.accLength}\n"
                f"Acceleration Heading: {self.accHeading}\n"
                f"Reserved2: {self.reserved2}\n"
                f"Flags: {self.flags}")
 


# Holds the latest telemetry received from the onboard PIC controller,
# the H2O sensor module, the sonar, and the IMU (see serial_segnali.py /
# handle_command, which fills these fields from the $PWR_/$H2O_/$SON_/etc. messages).
class signal_data:
    def __init__(self):
        self.stato_guida_automatica = None
        self.Volt_Batt_A = None
        self.Volt_Batt_B = None
        self.Volt_Batt_C = None
        self.Amp_Batt_A = None
        self.Amp_Batt_B = None
        self.Amp_Batt_C = None
        self.override_guida_automatica = None
        self.motore_A_dir = None
        self.motore_A_speed = None
        self.motore_B_dir = None
        self.motore_B_speed = None
        self.H2O_tds_temp = None
        self.H2O_ph = None
        self.H2O_tds_val = None
        self.H2O_oxygen = None
        self.ADC_valore_mediato = None
        self.ADC_valoreM_volt_alimentazione_sensori = None
        self.Sonar_val = None
        self.Gps_UTC = None
        self.Gps_latitude = None
        self.Gps_longitude = None
        self.Gps_altitude = None
        self.Gps_quality = None
        self.Gps_speed_overground = None # speed over ground (NMEA GNVTG "ssogk" field)
        self.Gps_degree_to_north = None # course over ground, true north (NMEA GNVTG "scogt" field)
        self.Imu_x = None
        self.Imu_y = None
        self.Imu_z = None
        self.Imu_z_fusione = None
        self.Info_countidpp = None
        self.Info_temp = None

# Boat navigation state: current position (from GPS), target waypoint,
# initial position, and the heading/distance error used by the autopilot
# loop in serial_motori.py.
class all_data:
    def __init__(self):

        self.raw_nmea_lat = None
        self.raw_nmea_lon = None
        self.lat_d = None
        self.lat_m = None
        self.lat_s = None
        
        self.lon_d = None
        self.lon_m = None
        self.lon_s = None
        
        self.lat_d_target = None
        self.lat_m_target = None
        self.lat_s_target = None
        
        self.lon_d_target = None
        self.lon_m_target = None
        self.lon_s_target = None

        self.lat_d_init = None
        self.lat_m_init = None
        self.lat_s_init = None
        
        self.lon_d_init = None
        self.lon_m_init = None
        self.lon_s_init = None


        self.lat_dir = None
        self.lon_dir = None

        self.lat_dd = None
        self.lon_dd = None

        self.lat_dd_target = None
        self.lon_dd_target = None

        self.lat_dd_init = None
        self.lon_dd_init = None


        self.tar_lat_dd = None
        self.tar_lon_dd = None
        self.head= 0.0
        self.head_error = 0.0
        self.distance = None
