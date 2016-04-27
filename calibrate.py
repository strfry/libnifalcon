from ctypes import cdll, c_float
lib = cdll.LoadLibrary('./pyfalcon.so')

cVec3 = c_float * 3
print "Initialize", lib.initialize(0)

run_io = lib.run_io
#lib.run_io.argtypes = [cVec3, cVec3]

lib.get_position.argtypes = [cVec3]

cpos = cVec3(0., 0., 0.11)
cforce = cVec3(0., 0., 0.)

## Buttons
# Change gradient / acceleration

import numpy as np

import time
cur_time = time.time()

min_pos = np.array([9999.] * 3)
max_pos = np.array([0.] * 3)


while True:	
	# Update Position
	lib.get_position(cpos)
	pos = np.array(cpos)

	min_pos = np.min((min_pos, pos), axis=0)
	max_pos = np.max((max_pos, pos), axis=0)
	

	force = [0., 0., 0.]	
	lib.set_force(cVec3(force[0], force[1], force[2]))

	run_io()

	time.sleep(0.01)

	total_volume = np.dot(max_pos - min_pos, max_pos - min_pos)
	
	print pos
	print "min =", min_pos
	print "max =", max_pos
	print "total_volume", total_volume
	if total_volume > 0.03: break

print "Please copy calibration values from above"
