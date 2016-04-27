from ctypes import cdll, c_float
lib = cdll.LoadLibrary('./pyfalcon.so')

cVec3 = c_float * 3


run_io = lib.run_io
#lib.run_io.argtypes = [cVec3, cVec3]

lib.get_position.argtypes = [cVec3]
#lib.set_force = cVec3

cpos = cVec3(0., 0., 0.11)
cforce = cVec3(0., 0., 0.)

import numpy as np


from pybrain.tools.shortcuts import buildNetwork






###############################################
# http://simontechblog.blogspot.de/2010/08/pybrain-reinforcement-learning-tutorial_15.html
# 
# Another take on Reinforcement learning

#Vec3 = np.float * 3


from pybrain.rl.environments import Environment
from pybrain.rl.environments.task import Task

class DildoEnvironment(Environment):
	indim = 3
	outdim = 6
	
	def __init__(self):
		Environment.__init__(self)
		print "Initialize", lib.initialize(0)
		self.position = np.zeros(3)

	def getSensors(self):
		return [self.position]

	def performAction(self, action):
		force = action
		lib.set_force(cVec3(force[0], force[1], force[2]))
		run_io()
		#CTypes Workaround for get_position
		cpos = cVec3()
		lib.get_position(cpos)
		self.position = np.array(cpos)
		

	def reset(self):
		""" Most environments will implement this optional method that allows for reinitialization.
		"""
		pass

    
		
class CenterTask(Task):
	def __init__(self, environment):
		Task.__init__(self, environment)
		self.environment = environment		
		
	def performAction(self, action):
		print "CenterTask Action", action
		self.environment.performAction(action)
		
	def getReward(self):
		pos = self.environment.position
		return 1.0 / np.dot(pos, pos)
		
	def getObservation(self):
		return self.environment.position
		
	@property
	def indim(self):
		return self.environment.indim

	@property
	def outdim(self):
		return self.environment.outdim
		


def build_network():
	from pybrain.structure import RecurrentNetwork
	from pybrain.structure import LinearLayer, SigmoidLayer, FullConnection

	net = RecurrentNetwork()
	net.addInputModule(LinearLayer(3, name='pos_in'))
	net.addModule(SigmoidLayer(3, name='hidden'))
	net.addOutputModule(LinearLayer(3, name='force_out'))

	net.addConnection(FullConnection(net['pos_in'], net['hidden']))
	net.addConnection(FullConnection(net['hidden'], net['force_out']))
	net.addRecurrentConnection(FullConnection(net['hidden'], net['hidden']))

	net.sortModules()
	return net
	
net = build_network()

# define action-value table
# number of states is:
#
#    current value: 1-21
#
# number of actions:
#
#    Stand=0, Hit=1av_table = ActionValueTable(21, 2)

from pybrain.rl.learners.valuebased.interface import ActionValueNetwork
from pybrain.structure.modules.module import Module

class MyNetwork(ActionValueNetwork, Module):
	indim = 3
	outdim = 3
		
	def __init__(self, net):
		self.network = net
		
		return Module.__init__(self, 3, 3)
		self.inputbuffer = [np.zeros(self.indim)]
		self.outputbuffer = [np.zeros(self.outdim)]
		
	def getActionValues(self, action):
		return [self.network.activate(action)]

#avn = ActionValueNetwork(3, None)
#def activate(self):
#	return self.network.activate(state)
#avn.getActionValues = activate
#avn.initialize(0.)

avn = MyNetwork(net)

# define Q-learning agent
from pybrain.rl.learners import Q, NFQ
from pybrain.rl.explorers.continuous import NormalExplorer
from pybrain.rl.agents import LearningAgent
from pybrain.rl.learners.directsearch.reinforce import Reinforce
#learner = Q(0.5, 0.0)
#learner = NFQ()
learner = Reinforce()
#learner.explorer = NormalExplorer(3)
#learner._setExplorer(EpsilonGreedyExplorer(0.0))
agent = LearningAgent(avn, learner)

# define the environment
env = DildoEnvironment()

# define the task
task = CenterTask(env)

# finally, define experiment
from pybrain.rl.experiments import ContinuousExperiment
experiment = ContinuousExperiment(task, agent)

# ready to go, start the process

agent.learning = False
experiment.doInteractionsAndLearn(100)
	

while True:
#if True:
	agent.learning = True
	experiment.doInteractionsAndLearn(1)
    #agent.learn()
    #agent.reset()

	
########################################################################



### Graveyard
 ################################################
#                   \\   //						#
#                    \\ //						#
#                     \X/						#
#                     /X\\						#
#                    //  \\						#
#                   //    \\					#
################################################

exit()

history = []


from pybrain.rl.environments import EpisodicTask

class CenterTask(EpisodicTask):
	def __init__(self, env):		
		EpisodicTask.__init__(self, env)
		self.maxPower = 100.0 #Overall maximal tourque - is multiplied with relative max tourque for individual joint to get individual max tourque
		self.reward_history = []
		self.count = 0 #timestep counter
		self.epiLen = 500 #suggestet episodic length for normal Johnnie tasks
		self.incLearn = 0 #counts the task resets for incrementall learning
		self.env.FricMu = 20.0 #We need higher friction for Johnnie
		self.env.dt = 0.01 #We also need more timly resolution
		
	def performAction(self, force):
		env.set_force(force)
		env.run_io()
		self.pos = env.get_position()
		EpisodicTask.performAction(self, force)
		
	def getReward(self):
		return 1.0 / np.dot(self.pos, self.pos)
		
	def denormalize(self, pos):
		#pos * (1 + min/pos + max - min)
		return pos + min + max * pos - min * pos
		
	def normalize(self, pos):
		############################################
		# Solve for x								#
		#											#
		# pos = x * (1 + min/x + max - min)			#
		#											#
		# pos = x + min + x max - x min				#
		# pos - min = x * (1 + max - min)			#
		# x = (pos - min) / (1 + max - min)			#
		 ############################################

		return (pos - min) / (np.array([1] * 3) + max - min)

	def isFinished(self):
		#returns true if episode timesteps has reached episode length and resets the task
		if self.count > 64:
			#self.res()
			return True
		else:
			self.count += 1
			return False
            
	def res(self):
		#sets counter and history back, increases incremental counter
		self.count = 0
		self.incLearn += 1
		self.reward_history.append(self.getTotalReward())


		# normalize standard sensors to (-1, 1)
		self.sensor_limits = []
		#Angle sensors
		for i in range(self.env.actLen):
			self.sensor_limits.append((self.env.cLowList[i], self.env.cHighList[i]))
		# Joint velocity sensors
		for i in range(self.env.actLen):
			self.sensor_limits.append((-20, 20))
		#Norm all actor dimensions to (-1, 1)
		#self.actor_limits = [(-1, 1)] * env.actLen
		self.actor_limits = None

while True:
	import time
	time.sleep(0.01)
	run_io()
	

	# Update Position
	lib.get_position(cpos)
	pos = np.array(cpos)
	
	force = np.zeros(3)
	
	force = net.activate(pos)
	
	
	history.append(pos)
	
	#if len(history) < HISTORY_LENGTH: 
		#print 'ready to learn'
		#history_list = np.array((map((lambda (p, f): p), history))).flatten()
		#print "history list", history_list
		#force = net.activate(history_list)
		#force *= 1000
	#	continue
		
		
	
	#print 'ready to learn'
	#history_list = np.array((map((lambda (p, f): p), history))).flatten()
	#print "history list", history_list
	#force = net.activate(pos)
	#force *= 1000
	
	lib.set_force(cVec3(force[0], force[1], force[2]))
	
	
	from pybrain.supervised import RPropMinusTrainer
	
	
	#print "Activate Force", force
	
	if len(history) == HISTORY_LENGTH:
		print 'Learning Step' 
		from pybrain.datasets.sequential import SequentialDataSet

		dataSet = SequentialDataSet(3, 3)
		for pos in history:
			dataSet.addSample(pos, pos)
			
		trainer = RPropMinusTrainer(net, dataset=dataSet, verbose=True )
		trainer.train()
		trainer.train()
		trainer.train()

		history.clear()
	
		
	#opti = ContinuousOptimizer(error, history)
	#opti.minimize = True
	#opti.learn()
	
		
	#ds.addSample(pos_history[-1], pos_history[-2], force_history[-1])
	
	#pos_history.append(pos)
	#force_history.append(force)
	

print "Starting Experiments"
while True:
    experiment.doInteractions(100)
    agent.learn()
    agent.reset()
