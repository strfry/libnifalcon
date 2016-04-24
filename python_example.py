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

while True:
	run_io()
	
	#Save Last Position
	last_pos = pos
	
	# Update Position
	lib.get_position(cpos)
	pos = np.array(cpos)
	
	# Compute Normal Vector
	direction = pos / np.sum(pos)
	
	# Calculate Differential
	gradient = pos - last_pos
	#force = abs(pos) * -direction - gradient
	
	damping = 1
	force = gradient * damping # Acceleration, Damping Friction
	
	#force += abs(pos) * -direction * .1
	
	#force[0] = np.sin(cnt / 12) * 10
		
	lib.set_force(cVec3(force[0], force[1], force[2]))
	
	# Log Limiting Counter
	cnt += 1
	if cnt % 40 == 0: # 25 Hz Update Rate
		print pos[0], pos[1], pos[2],
		print "force", force[0]
