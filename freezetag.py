import time

from map_server import MapServer, IT, RUNNER
from robot import Robot

from read_config import read_config


if __name__ == '__main__':
	config = read_config()

	mServer = MapServer()

	robots = []
	for i in range(12):
		robots.append(mServer.getRobot(RUNNER))
	it = mServer.getRobot(IT)
	#robots.append(it)

	step = 0
	while len(mServer.runners) != 0:
		for r in robots:
			r.step()
		mServer.vis.refresh()
		it.step()
		it.step()
		mServer.vis.refresh()
		#time.sleep(2)


	print 'done'
	time.sleep(10)