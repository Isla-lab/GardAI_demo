import serial
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),"..")))
from functions.Serials_functions.functions_external import *


# Reads the dual-antenna GPS serial stream, which multiplexes two message
# types (see Documentation/Barca-2024.pdf, "GPS doppia Antenna"):
#  - binary UBX_NAV_RELPOSNED messages, starting with sync byte 0xB5, giving heading
#  - NMEA text sentences ($GNGGA, $GNVTG, $GNGST, ...), starting with '$', giving position
def read_serial(ser,Dati,antenna):
	time.sleep(1)
	ser.flushInput()
	while True:
		header = ser.read(1)
		if header == b'\xb5':
			# UBX binary message: read the rest of the 72-byte RELPOSNED payload
			# and pull the fused heading (relPosHeading, scaled by 1e5 deg) out of it.
			binary_data = header + ser.read(71)
			binary_data = binary_data.hex().upper()
			antenna.update_values(str(binary_data[12:]))

			heading =  (float(antenna.relPosHeading)/100000)
			Dati.head = heading
		if header == b'$':
			value = ser.readline()
			# Only the GNGGA sentence (position fix) is used here; other NMEA
			# sentences on this same line (GNVTG, GNGST, ...) are ignored.
			if value[:5]== b'GNGGA':

				latitude,dir_lat,longitude,dir_lon = lat_lon(str(value))
				Dati.raw_nmea_lat = latitude
				Dati.raw_nmea_lon = longitude
				Dati.lat_dd = convert_ddm_to_dd(latitude)
				Dati.lon_dd = convert_ddm_to_dd(longitude)
				Dati.dir_lat = dir_lat
				Dati.dir_lon = dir_lon
