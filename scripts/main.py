from flask import Flask, jsonify, render_template
import threading
import serial
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from seriali.serial_doppia_antenna import *
from seriali.serial_motori import *
from seriali.serial_segnali import *
from classes.classes_externals import *



Signal_data = signal_data()  # shared telemetry state, read by the Flask /data route

# NOTE: hardcoded Linux path - only works on the boat's original machine.
TEMPLATE_PATH = os.path.abspath("/home/francesco-univr/Desktop/Borsa/GardAI_9_04/Garda_9_04_25_gardai/templates")
app = Flask(__name__,template_folder=TEMPLATE_PATH)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/data')
def get_segnale_data():
    return jsonify({
        'Sonar_val': Signal_data.Sonar_val,
        'H2O_ph': Signal_data.H2O_ph,
        'H2O_tds_temp': Signal_data.H2O_tds_temp,
		'H2O_oxygen': Signal_data.H2O_oxygen,
		'H2O_tds_val': Signal_data.H2O_tds_val
	})

if __name__ == "__main__":

	# Three physical serial connectors on the boat (see Documentation/Barca-2024.pdf,
	# "INTERFACCIA a bordo"): dual-antenna GPS, motor controller, PIC/sensor telemetry.
	antenna  = DoppiaAntenna()
	ser_antenna = serial.Serial('/dev/GPS')
	ser_antenna.baudrate = 115200
	ser_antenna.parity = serial.PARITY_NONE
	ser_antenna.stopbits = serial.STOPBITS_ONE

	ser_write = serial.Serial('/dev/MOTORI')
	ser_write.baudrate = 57600
	ser_write.parity = serial.PARITY_NONE
	ser_write.stopbits = serial.STOPBITS_ONE
 
	ser_segnali = serial.Serial('/dev/Dati')
	ser_segnali.baudrate = 57600
	ser_segnali.parity = serial.PARITY_NONE
	ser_segnali.stopbits = serial.STOPBITS_ONE

	Dati = all_data()
	# read_thread: GPS/heading input, write_thread: autopilot + motor output,
	# signal_thread: PIC/sensor telemetry input, flask_thread: web dashboard.
	read_thread = threading.Thread(target=read_serial, args=(ser_antenna, Dati,antenna))
	signal_thread = threading.Thread(target=read_signal, args=(ser_segnali,Signal_data))
	write_thread = threading.Thread(target=write_serial, args=(ser_write, Dati, Signal_data))
	flask_thread = threading.Thread(target=lambda: app.run(debug=True,host= '192.168.95.130',port = 5000,use_reloader=False))

	read_thread.start()
	signal_thread.start()
	write_thread.start()
	flask_thread.start()

	read_thread.join()
	signal_thread.join()
	write_thread.join()
	flask_thread.join()
