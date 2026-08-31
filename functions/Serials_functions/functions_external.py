import math
import sys
import os
import numpy as np
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from classes.classes_externals import *
from functions.Serials_functions import hex_functions as hexconv



# Parses one comma-separated telemetry line coming from the onboard PIC
# controller / sensor boards (protocol documented in Documentation/Barca-2024.pdf)
# and stores the values on Dati/signal_data. Each PIC message type ($AUTO_, $SON_,
# $PWR_, $H2O_, $IMU_, $GPS_, $INFO_, $ADC_) has its own field layout.
def handle_command(command,Dati,lista_parametri):
    # Define functions for each command
    def handle_auto():
        Dati.stato_guida_autonoma = lista_parametri[1]
        Dati.override_guida_automatica = lista_parametri[2]
        Dati.motore_A_dir = lista_parametri[3]
        Dati.motore_A_speed = lista_parametri[4]
        Dati.motore_B_dir = lista_parametri[5]
        Dati.motore_B_speed = lista_parametri[6]
        return "Handling AUTO command"

    def handle_son():
        Dati.Sonar_val =  lista_parametri[1]
        return "Handling SON command"

    def handle_pwr():
        Dati.stato_guida_autonomatica = lista_parametri[1]
        Dati.Volt_Batt_A = lista_parametri[2]
        Dati.Volt_Batt_B = lista_parametri[3]
        Dati.Volt_Batt_C = lista_parametri[4]
        Dati.Amp_Batt_A  = lista_parametri[5]
        Dati.Amp_Batt_B  = lista_parametri[6]
        Dati.Amp_Batt_C  = lista_parametri[7]
        return "Handling PWR command"

    def handle_h2o():
        Dati.H2O_tds_temp = lista_parametri[1]
        Dati.H2O_ph = lista_parametri[2]
        Dati.H2O_tds_val = lista_parametri[3]
        Dati.H2O_oxygen = lista_parametri[4]
        return "Handling H2O command"

    def handle_imu():
        Dati.Imu_x = lista_parametri[1]
        Dati.Imu_y = lista_parametri[2]
        Dati.Imu_z = lista_parametri[3]
        Dati.Imu_z_fusione = lista_parametri[4]
        return "Handling IMU command" + str(Dati.Imu_x) + " " + str(Dati.Imu_y) + " " + str(Dati.Imu_z) + " " + str(Dati.Imu_z_fusione)

    def handle_gps():
        Dati.Gps_UTC = lista_parametri[1]
        Dati.Gps_latitude = lista_parametri[2]
        Dati.Gps_longitude = lista_parametri[3]
        Dati.Gps_altitude = lista_parametri[4]
        Dati.Gps_quality = lista_parametri[5]
        Dati.Gps_speed_overground = lista_parametri[6] # ssogk
        Dati.Gps_degree_to_north = lista_parametri[7] #scogt
        return "Handling GPS command" + str(Dati.Gps_latitude) + " " + str(Dati.Gps_longitude) + " " + str(Dati.Gps_altitude)

    def handle_info():
        Dati.countidpp = lista_parametri[1]
        Dati.Gps_UTC =lista_parametri[2]
        Dati.Gps_latitude =lista_parametri[3]
        Dati.Gps_longitude =lista_parametri[4]
        Dati.Info_temp = lista_parametri[5]
        Dati.H2O_ph =lista_parametri[6]
        Dati.Info_conduct = lista_parametri[7]
        Dati.H2O_oxygen =lista_parametri[8]
        Dati.Info_profondità =lista_parametri[9]
        Dati.Info_automatico =lista_parametri[10]
        Dati.Info_Va =lista_parametri[11]
        Dati.Info_Vb =lista_parametri[12]
        Dati.Info_Vc =lista_parametri[13]
        Dati.Info_speed =lista_parametri[14]
        return "Handing INFO command"
    
    def handle_adc():
        Dati.ADC_valore_mediato = lista_parametri[1]
        Dati.ADC_valoreM_volt_alimentazione_sensori = lista_parametri[2]
        return "Handing ADC command"
    # Define a dictionary to map commands to their corresponding functions
    command_actions = {
        "$AUTO_": handle_auto,
        "$SON_": handle_son,
        "$PWR_": handle_pwr,
        "$H2O_": handle_h2o,
        "$IMU_": handle_imu,
        "$GPS_": handle_gps,
        "$INFO_": handle_info,
        "$ADC_" : handle_adc,
    }


    # Get the action based on the command, or return a default message if the command is not found
    action = command_actions.get(command, lambda: "Unknown command" + str(command))

    # Execute the action and return the result
    return action()



def lat_lon(input_string):
    # input_string is str(bytes_from_serial), e.g. "b'GNGGA,093642.00,4518.76...,N,...\\r\\n'"
    # strip() removes any of these individual characters from both ends (not a
    # literal substring match) - this only works because it matches the edges
    # of Python's bytes repr() output exactly.
    clean_string = input_string.strip("b'").strip("\\r\\n'")
    # Split the NMEA GGA sentence by comma
    parts = clean_string.split(',')

    # GGA field order: sentence_id,UTC,lat,N/S,lon,E/W,... -> lat=idx2, lon=idx4
    latitude = parts[2]
    dir_latitude = parts[3]
    longitude = parts[4]
    dir_longitude = parts[5]
    return latitude, dir_latitude,longitude,dir_longitude


# NMEA sends coordinates as DDM (degrees + decimal minutes, e.g. "4518.7635264"),
# not decimal degrees - this converts that to standard DD.
# Note: does not apply hemisphere sign (N/S, E/W) - fine as long as the boat
# stays in the northern/eastern hemisphere (Lake Garda), not portable as-is.
def convert_ddm_to_dd(ddm_str):
    # Determine where the decimal point is
    dot_index = ddm_str.index(".")
    
    # If there are 5 digits before the decimal, take the first 3 as degrees
    if dot_index == 5:
        degrees = int(ddm_str[:3])
        minutes = float(ddm_str[3:])
    else:  # If there are 4 digits before the decimal, take the first 2 as degrees
        degrees = int(ddm_str[:2])
        minutes = float(ddm_str[2:])
    
    # Convert to decimal degrees
    decimal_degrees = degrees + minutes / 60
    return decimal_degrees

   
# Linear rescale of value from [old_min, old_max] into [new_min, new_max].
# Used to map computed motor thrust (-4.5..4.5) into the PIC's -1000..1000 range.
def rescale(value, old_min, old_max, new_min, new_max):
      # Calculate the ratio of the value in the old range
    ratio = (value - old_min) / (old_max - old_min)
    
    # Calculate the new value in the new range
    new_value = new_min + (ratio * (new_max - new_min))
    
    return new_value

    
    
# Converts a raw NMEA DDM coordinate string into (degrees, minutes, seconds),
# used to populate the human-readable DMS fields of Dati (target waypoints).
def gps_to_dms(gps_string):
    if gps_string:
        gps_string = gps_string.lstrip('0')
        degrees = int(gps_string[:2])  # Extract the degrees (first 2 characters)
        minutes_decimal = float(gps_string[2:])  # Convert remaining part to float for minutes
        minutes = int(minutes_decimal)  # Integer part is the minutes
        seconds = (minutes_decimal - minutes) * 60  # Decimal part times 60 gives seconds

        return degrees, minutes, seconds
    else:
        return 0,0,0   
    
    

# Builds a $<cmd>*<checksum>\r\n frame for the PIC motor-control protocol.
# Checksum is the NMEA-style XOR of all bytes in cmd, written as 2 hex digits.
def update_checksum(cmd):
    checksum = 0
    for char in cmd:
        checksum ^= ord(char)
    hexsum = format(checksum, '02X')
    result = f"${cmd}*{hexsum}\r\n"
    return result


# Great-circle distance between two lat/lon points (decimal degrees), in meters.
# Used as the "distance to target waypoint" error term for the autopilot.
def haversine(lat1, lon1, lat2, lon2):

    R = 6371000  # Earth radius in meters
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    
    d_lat = lat2 - lat1
    d_lon = lon2 - lon1
    
    a = math.sin(d_lat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(d_lon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    return R * c  # Distance in meters

# Builds the PIC motor command payload: $&_1_1_dirDX_speedDX_dirSX_speedSX_*crc
# (see Documentation/Barca-2024.pdf, "PIC" section). dir: 0=stop,1=forward,2=reverse.
def get_motor_string(stato_dx,stato_sx,speed_dx,speed_sx):
    return "&_1_1_"+ str(stato_dx) + "_" + str(speed_dx) + "_" + str(stato_sx) + "_" + str(speed_sx) + "_"


def get_opposite_angle(angle):
    return (angle + 180) % 360

# Returns 1 if test_angle lies on the right side of given_angle, 0 if on the left.
# Used by the autopilot to decide which way to turn towards the target bearing.
def is_on_right_or_left(given_angle, test_angle):
    # Normalize angles
    given_angle = given_angle % 360
    test_angle = test_angle % 360
    opposite_angle = get_opposite_angle(given_angle)

    # If the test_angle is between given_angle and opposite_angle
    if given_angle < opposite_angle:
        if given_angle < test_angle < opposite_angle:
            return 1 #right
        else:
            return 0 #left
    else:
        # This handles the wraparound at 360 degrees
        if given_angle < test_angle or test_angle < opposite_angle:
            return 1
        else:
            return 0

# Smallest absolute angular gap between two headings (0-180 degrees),
# used by the autopilot to decide when it's close enough to stop turning.
def angle_difference(angle1, angle2):
    # Normalize both angles to the range [0, 360)
    angle1 = angle1 % 360
    angle2 = angle2 % 360

    # Find the absolute difference between the two angles
    diff = abs(angle1 - angle2)

    # Return the smallest angle difference, considering the wrap-around at 360 degrees
    return min(diff, 360 - diff)

# Note: bearing_radians, bearing_degrees_180 and reverse_bearing_180 below are
# computed but not returned/used - only bearing_degrees feeds the return value.
def initial_bearing(lat1, lon1, lat2, lon2):
    """
    Calculate the initial bearing (forward azimuth) from point 1 to point 2,
    i.e. the compass heading (0-360) the boat should follow to reach point 2.
    """
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    
    dlon = lon2 - lon1
    
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
    
    bearing = math.atan2(x, y)
    bearing_degrees = math.degrees(bearing)
    bearing_radians = ((bearing + math.pi) % (2 * math.pi)) - math.pi  # Convert to -pi to +pi format
    bearing_degrees_180 = ((bearing_degrees + 180) % 360) - 180  # Convert to -180 to +180 format

    bearing_int = (bearing_degrees + 360) % 360
    reverse_bearing = (360-bearing_int) % 360  # Reverse bearing calculation
    reverse_bearing_180 = ((reverse_bearing + 180) % 360) - 180  # Reverse bearing in -180 to 180 format
    
    return (bearing_degrees + 360) % 360#, bearing_radians, bearing_degrees_180, reverse_bearing, reverse_bearing_180



