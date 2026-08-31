import serial
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import time
import numpy as np
import math as math
import csv
from datetime import datetime

from classes.classes_externals import *
from functions.Serials_functions.functions_external import *


# Waypoint-following autopilot: steers the boat through a fixed list of GPS
# targets by alternating between "turn to face the target" (status 1) and
# "go straight" (status 2), computing motor thrust from the heading/distance
# error each loop iteration, and logging every step to boat_log.csv.
def write_serial(ser_write,Dati,Signal_data):
	time.sleep(5)

	ser_write.flushOutput()
	# Initialize the CSV log file outside the loop
	csv_file = "boat_log.csv"
	file_exists = os.path.isfile(csv_file)
	with open(csv_file, mode='a', newline='') as f:
		writer = csv.writer(f)
		if not file_exists:
			# Create the header row if the file doesn't exist yet
			writer.writerow(["Timestamp", "Raw_NMEA_Lat", "Raw_NMEA_Lon", "Lat_DD", "Lon_DD", "Cmd_Motori", "Temp", "Sonar"])
	index = 0
	# Waypoints as raw NMEA DDM strings (lat, lon pairs), visited in order and looped
	target =["4521.6065846","01101.0665822","4521.6090789","01101.0605647","4521.6030600","01101.0596158"]
	status=1
	while True:
		Dati.lat_d_target,Dati.lat_m_target,Dati.lat_s_target = gps_to_dms(target[index])
		Dati.lon_d_target,Dati.lon_m_target,Dati.lon_s_target = gps_to_dms(target[index+1])
  
		Dati.lat_dd_target = convert_ddm_to_dd(target[index])
		Dati.lon_dd_target = convert_ddm_to_dd(target[index+1])

		error_linear = haversine(Dati.lat_dd, Dati.lon_dd,Dati.lat_dd_target,Dati.lon_dd_target)
		target_bearing = initial_bearing(Dati.lat_dd, Dati.lon_dd,Dati.lat_dd_target,Dati.lon_dd_target) 
		current_bearing = Dati.head
		error_angular = target_bearing - current_bearing
		Dati.head_error = error_angular
		Dati.distance = error_linear
		vel = 1.0
		time.sleep(0.1) # decomment for ability to regain control
		# ALLIGN
		print('Current: '+ str(index) + 'Linear: :' + str(error_linear)+ 'Angular '+str(error_angular))

		if status == 1: # turning to face the target bearing
			if is_on_right_or_left(current_bearing,target_bearing)==1: # its on the right -> left motor power
				trust_R = +vel*0.5
				trust_L = -vel*0.5
			elif -is_on_right_or_left(current_bearing,target_bearing)==0: # its on the left -> right motor power
				trust_R = -vel*0.5
				trust_L = +vel*0.5
			# trust_R = -vel*0.5
			# trust_L = +vel*0.5
			if angle_difference(current_bearing,target_bearing) < 30:
				
				# trust_R = 0.0
				# trust_L = 0.0
				# time.sleep(3.0)
				status = 2
				

		if status == 2: # go straight
			
			if error_linear > 3.0:
				trust_L = vel*3.0
				trust_R = vel*3.0
			else:
				trust_L = vel*1.0
				trust_R = vel*1.0
			# COomment this
			trust_L = vel*0.7
			trust_R = vel*0.7
			if angle_difference(current_bearing,target_bearing) > 30:
				status = 1
		
		if status == 3: # stop (unreachable in the current loop, kept for manual/future use)
			trust_L = vel*0.0
			trust_R = vel*0.0
		
		trust_L = rescale(trust_L, -4.5, 4.5, -1000, 1000)
		trust_R = rescale(trust_R, -4.5, 4.5, -1000, 1000)

		if trust_L > 1000:
			trust_L = 1000

		if trust_L < -1000:
			trust_L = -1000

		if trust_R > 1000:
			trust_R = 1000

		if trust_R < -1000:
			trust_R = -1000


		if trust_L > 0:
			stato_motore_dx = 1
		elif trust_L < 0:
			stato_motore_dx = 2
		else:
			stato_motore_dx = 0

		if trust_R > 0:
			stato_motore_sx = 1
		elif trust_R < 0:
			stato_motore_sx = 2
		else:
			stato_motore_sx = 0

		speed_motore_dx = abs(trust_R)
		speed_motore_sx = abs(trust_L)

		#stato_motore_dx = 1
		#stato_motore_sx = 1
		#speed_motore_dx = 0
		#speed_motore_sx = 0
		# #change if reached was 1 before
		if error_linear < 1.5:
			index = index + 2
		

		#print(index)
		# # #restart if reached end
		if index == 6:
			index = 0
		print("Status" + str(status) + "Errore lineare: " + str(error_linear))
		command_motors = get_motor_string(stato_motore_dx,stato_motore_sx,speed_motore_dx,speed_motore_sx)
		# print(str(current_bearing) + '-' + str(target_bearing))
		print(index)
		# print(str(trust_L) + ' AND ' + str(trust_R))
		#print(status)
		ser_write.write(str.encode(update_checksum(command_motors)))

		timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

		# Safely read values for logging: default to 0 if still None (e.g. no GPS fix yet)
		raw_lat = Dati.raw_nmea_lat if getattr(Dati, 'raw_nmea_lat', None) is not None else 0
		raw_lon = Dati.raw_nmea_lon if getattr(Dati, 'raw_nmea_lon', None) is not None else 0
		lat_dd = Dati.lat_dd if Dati.lat_dd is not None else 0
		lon_dd = Dati.lon_dd if Dati.lon_dd is not None else 0

		# command_motors is the string just sent to the motors (0 if somehow empty)
		cmd = command_motors if command_motors else 0

		temp = Signal_data.H2O_tds_temp if Signal_data.H2O_tds_temp is not None else 0
		sonar = Signal_data.Sonar_val if Signal_data.Sonar_val is not None else 0

		# Write the log row at the end of this loop iteration
		with open(csv_file, mode='a', newline='') as f:
			writer = csv.writer(f)
			writer.writerow([timestamp, raw_lat, raw_lon, lat_dd, lon_dd, cmd, temp, sonar])
