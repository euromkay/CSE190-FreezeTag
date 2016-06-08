#!/usr/bin/env python
from read_config import read_config
from laser_sensor import *
import math
import time
from texture_sensor import TexSensor
from temperature_sensor import TempSensor
import random
import numpy as np
import mdp

C_GRAD = [31, 91, 33, 93, 32, 92]
runningForever = True

def sleep(wait):
	if True:
		time.sleep(wait)

class Robot():
	def __init__(self, mapServer):
		self.mapServer = mapServer
		self.textSensor = TexSensor(self)
		self.tempSensor = TempSensor(self)
		self.laserSensor = LaserSensor(self)
		self.lastMove = [1,0]
		self.notEnded = True
		self.mover = self.mapServer.handle_move_request
		self.error = 0
		self.count = 0

		

	def setupConfig(self):

		config = read_config()
		self.config = config
		self.sig              = config['temp_noise_std_dev']
		self.moveList         = config['move_list']
		self.energy           = config['starter_energy']
		self.height           = self.mapServer.height
		self.width            = self.mapServer.width

		random.seed(len(self.mapServer.vis.robots) + self.config['seed'])
		self.pos              = self.mapServer.grid.get(random.randrange(self.width), random.randrange(self.height))
		self.pos.robots.append(self)
		

		self.text_probability = config['prob_tex_correct']
		self.move_success     = config['prob_move_correct'] if config['uncertain_motion'] else 1
		self.move_fail        = (1 - self.move_success)/(len(config['possible_moves']) - 1.)  if config['uncertain_motion'] else 0
		init_probability = 1./(self.height * self.width)
		for p in self.grid.getPoints():
			p.prob[self] = init_probability
		

		self.updateTemp()
		self.updateTexture()

		self.possible_moves = [move for move in config['possible_moves']]
		


	def printmap(self):
		#print 'actual_pos : ' + str(self.pos)

		ret = []
		for h in range(self.height):
			row = []
			for w in range(self.width):
				#row.append(self.grid.get(w,h).prob[self])
				row.append(self.grid.get(w,h).value/100)
			ret.append(row)
		print_2d_floats(ret)

	def publishAll(self, temp, text):
		self.probPublisher.publish(flatten(self.map))
		self.tempPublisher.publish(temp)
		self.textPublisher.publish(text)


	def step(self):
		self.updateTemp()
		self.updateTexture()
		
		if self.noMoreMoves():
			if self.notEnded:
				#self.publishAll(temp, text)
				self.endSimulation()
				self.notEnded = False
			return
		self.move()

		#self.publishAll(temp, text)

		if self.noMoreMoves():
			sleep(4)


	def laserScan(self):
		closest = self.laserSensor.scan()

	def printCoord(self):
		r_pos = self.getBelief()
		a_pos = self.actual_pos
		diff_x = min(abs(r_pos[0] - a_pos[0]), abs(r_pos[0] + self.width - a_pos[0]), abs(r_pos[0] - self.width - a_pos[0]))
		diff_y = min(abs(r_pos[1] - a_pos[1]), abs(r_pos[1] + self.height - a_pos[1]), abs(r_pos[1] - self.height - a_pos[1]))
		error = (diff_y ** 2 + diff_x ** 2) ** .5
		self.count += 1
		self.error *= float(self.count - 1)/(self.count)
		error *= 1.0/self.count
		self.error += error
		#print r_pos
		#print a_pos
		print self.error



	def updateTemp(self):
		temperature_read = self.tempSensor.sense()
		for p in self.grid.getPoints():
			actual_temperature = self.tempSensor.temp_dict[p.temp]
			p_temp_giv_x = pdf(self.sig, temperature_read, actual_temperature)
			p.prob[self] *= p_temp_giv_x

		self.normalizeMap()

	def updateTexture(self):
		texture_read = self.textSensor.sense()
		for p in self.grid.getPoints():
			actual_texture = p.text

			p_text_giv_x = self.text_probability if texture_read == actual_texture else (1 - self.text_probability)
			p.prob[self] *= p_text_giv_x
		
		self.normalizeMap() 

	def normalizeMap(self):
		factor = sum(p.prob[self] for p in self.grid.getPoints())
		for p in self.grid.getPoints():
			p.prob[self] /= factor

		



	def getXGivenTemp(self, temperature_read, actual_temperature, P_x):
		p_temp_given_x = pdf(self.sig, temperature_read, actual_temperature)
		top = p_temp_given_x * P_x

		all_temps = flatten(self.temp_map)
		p_temp_given_x_list = [pdf(self.sig, temperature_read, actual_temp) for actual_temp in all_temps]

		prev_loc_belief = flatten(self.map)
		bottom = [x * y for x,y in zip(prev_loc_belief, p_temp_given_x_list)]
		return top / sum(bottom)

	def endSimulation(self):
		sleep(4)
		sleep(3)

	def noMoreMoves(self):
		return self.moveList == []

	def getBelief(self):
		a = np.array([[n.prob[self] for n in col] for col in self.mapServer.grid.base])

		return np.amax(a), np.unravel_index(a.argmax(), a.shape)


	def getMove(self):
		belief = self.getBelief()
		pos = self.grid.get(belief[1][0], belief[1][1])
		#print pos
		if self.getBelief()[0] < 0.4:
			return random.choice(self.config['possible_moves'])

		
		mdp.mdp(self)
		return pos.policy
		

		bestMove = self.config['possible_moves'][0]
		bestReward = sum([mdp.expectedReward(n, bestMove, self.config, self) * n.prob[self] for n in self.grid.getPoints() if not n.isWall])
		

		for move in self.config['possible_moves'][1:]:
			reward = sum([mdp.expectedReward(n, move, self.config, self) * n.prob[self] for n in self.grid.getPoints() if not n.isWall])
			
			if reward > bestReward:
				bestReward = reward
				bestMove = move

		return bestMove




	def move(self):
		if self.noMoreMoves() or self.frozen:
			return

		the_move = self.getMove()
		if the_move != [0,0]:
			self.lastMove = the_move
		
		self.mover(the_move, self)
		#print self.pos

		#update beliefs
		

		for p in self.grid.getPoints():
			p.prob_new[self] = 0.0

		for p in self.grid.getPoints():
			old_val = p.prob[self] * (self.move_success - self.move_fail)

			move_point = p.move(the_move[0], the_move[1])
			move_point.prob_new[self] += old_val

			old_val = p.prob[self] * self.move_fail
			for a_move in self.possible_moves:
				move_point = p.move(a_move[0], a_move[1])
				move_point.prob_new[self] += old_val

		for p in self.grid.getPoints():
			p.prob[self] = p.prob_new[self]




		

		





		

def pdf(stdv, x, mu):
	top = x - mu
	top = top * top * -1.
	bottom = 2. * stdv * stdv
	e = math.e ** (top / bottom)
	total = e / stdv
	total = total / math.sqrt(2. * math.pi)
	return total

def flatten(my_map):
	ret = []
	for row in my_map:
		for x in row:
			ret.append(x)
	return ret

def map_2d(f, a):
    return [[f(c) for c in r] for r in a]

def zipwith(f, a, b):
    return [f(ac, bc) for ac, bc in zip(a, b)]

def zipwith_2d(f, a, b):
    return [zipwith(f, ar, br) for ar, br in zip(a, b)]

def color(code, s):
    return '\033[{}m{}\033[00m'.format(code, s)

def print_2d_floats(a):
    rowline = '\n+{}\n'.format(('-' * 5 + '+') * len(a[0]))
    print(rowline + rowline.join(
        '|{}|'.format('|'.join(
            color(C_GRAD[min(len(C_GRAD) - 1, int(c * 2 * len(C_GRAD)))],
                  '{:1.3f}'.format(c)) for c in r
        ))
    for r in a) + rowline)

if __name__ == '__main__':
	robot = Robot()
	while True:
		robot.step()
		sleep(1)