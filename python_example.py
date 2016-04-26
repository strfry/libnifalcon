from ctypes import cdll, c_float
lib = cdll.LoadLibrary('./pyfalcon.so')


import numpy as np

cVec3 = c_float * 3

print "Initialize", lib.initialize(0)

run_io = lib.run_io
#lib.run_io.argtypes = [cVec3, cVec3]

lib.get_position.argtypes = [cVec3]


cpos = cVec3(0., 0., 0.11)
cforce = cVec3(0., 0., 0.)

cnt = 0

last_pos = []
pos = np.array(cpos)

## Buttons
# Change gradient / acceleration

import time

cur_time = time.time()

history = []

phase = 0

while True:	
	# Save Time Delta
	last_time = cur_time
	cur_time = time.time()
	delta_time = cur_time - last_time
	
	#Save Last Position
	last_pos = pos
	# Update Position
	lib.get_position(cpos)
	pos = np.array(cpos)
	
	delta_pos = pos - last_pos
	
	# Calculate Differential
	gradient = delta_pos / delta_time
	
	# Compute Normal Vector
	center_distance = np.sqrt(np.dot(pos, pos))
	normal_vec = pos / center_distance
	
		
	# PID Regulator
	P = 60.0 * 5
	D = -1.9 * 10
	I = 30.5
	phase += 2 * np.pi * delta_time
	
	
	dildo_gravity = np.array([0, 1, 0]) * 9.81 * .4
	dildo_sinus = np.array([0, 1, 0])* 1000.0
	target_position = np.sin(dildo_sinus * phase * 20)
	target_position = np.zeros(3)
	
	target_position[1] = 0.05
	target_position[1] = 0.05 * np.sin(phase / 10.71) * 0.3 + 0.01
	#target_position = [0, np.sin(phase) * 0.0011, 0]
	
	difference = target_position - pos
	
	### Force Feedback
	
	#force = normal_vec * 1.21 * P + gradient * np.log(gradient) / np.log(8) * 0.1 * D # * P - gradient * D#- gradient
	
	#difference_gradient = difference / delta_time
	#force = difference * P + difference_gradient * D
	
	force = dildo_gravity + difference * P + difference / delta_time * D + difference * delta_time * I
	print type(pos), type(difference), type(force)
	force *= 0.001
	
	lib.set_force(cVec3(force[0], force[1], force[2]))

	
	## Archive / PID stuff
	
	#from math import sin
	#force[2] = sin(cnt / 8.) * 0.01
	#force[0] += sin(cnt/16) * 0.002
	
	damping = 0.01
	#force = gradient * damping # Acceleration, Damping Friction
	
	#force += abs(pos) * -direction * .1
	
	#force[0] = np.sin(cnt / 12) * 10
		

	run_io()
	
	# Log Limiting Counter
	cnt += 1
	if cnt % 40 == 0: # 25 Hz Update Rate
		print "Distance", center_distance
		print "Delta Time", delta_time, "Force Vector", force
