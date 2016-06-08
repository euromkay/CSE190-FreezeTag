import time
from read_config import read_config
from copy import deepcopy

def initialize(config, robot):

	for n in robot.grid.getPoints():
		if n.isWall:
			n.prevValue = None
			n.value = 0.0
		else:
			n.value = 0.0
			n.prevValue = 0.0

def probabilisticMoves(move, config):
	ret = [(move, config['prob_move_correct'])]

	possible_moves = deepcopy(config['possible_moves'])
	possible_moves.remove(move)
	for m in possible_moves:
		ret.append([m, ((1 - config['prob_move_correct'])/len(possible_moves))])

	return ret

def bel_sub(prob, reward, discount, utility):
	return prob * (reward + (discount * utility))

def rewardFunction(result_n, robot):
	config = robot.config
	reward = result_n.energy
	if robot.it:
		reward += sum([rob.energy for rob in result_n.robots if not rob.frozen and rob is not robot]) * config['freeze_reward']
	else:
		reward += sum([rob.mapServer.it.energy for rob in result_n.robots if rob.frozen]) * config['un_freeze_reward']
	
		it = robot.mapServer.it
		it_moves = []
		for move in config['possible_moves']:
			pos = it.pos.move(move[0], move[1])
			if pos is result_n:
				reward -= robot.energy * config['freeze_reward']
	return reward


def expectedReward(n, move, config, robot):
	results = [(r, n.move(r[0][0], r[0][1])) for r in probabilisticMoves(move, config)]
	ret = 0.0
	for r in results:
		action = r[0][0]
		result_n = r[1]
		probability = r[0][1]
		reward = rewardFunction(result_n, robot)

		ret += bel_sub(probability, reward, config['discount_factor'], result_n.prevValue)
	return ret


def reward_key(x):
	return x[1]

def iteration(grid, config, robot):
	config['max_iterations'] -= 1
	for n in grid.getPoints():
		if not n.isSpecial():
			n.prevValue = n.value
	difference = 0.0
	for n in grid.getPoints():
		if n.isWall:
			continue
		#print n
		move_reward_list = [(move, expectedReward(n,move, config, robot)) for move in config['possible_moves']]
		#print '\t' + str(move_reward_list)
		pair = max(move_reward_list, key = reward_key)
		n.value = pair[1]
		n.policy = pair[0]
		difference += abs(n.value - n.prevValue) 
	#grid.printGrid()
	#robot.printmap()
	#time.sleep(3)
	if difference < config['threshold_difference']:
		return False
	return config['max_iterations'] > 0

def mdp(robot):
	grid = robot.mapServer.grid
	config = read_config()
	initialize(config, robot)

	while iteration(grid, config, robot):
		pass


		
